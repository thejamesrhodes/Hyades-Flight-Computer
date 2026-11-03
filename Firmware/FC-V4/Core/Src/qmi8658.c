/* ============================================================================
 * qmi8658.c  —  QST QMI8658A driver (I2C, latest-sample mode). See qmi8658.h.
 * Register references: QMI8658A Datasheet Rev A.
 * ============================================================================ */
#include "qmi8658.h"
#include <string.h>

#define REG_WHO_AM_I      0x00u
#define REG_REVISION_ID   0x01u
#define REG_CTRL1         0x02u   /* SIM | ADDR_AI | BE | INT2_EN | INT1_EN | FIFO_INT_SEL | - | SensorDisable */
#define REG_CTRL2         0x03u   /* aST | aFS[6:4] | aODR[3:0] */
#define REG_CTRL3         0x04u   /* gST | gFS[6:4] | gODR[3:0] */
#define REG_CTRL5         0x06u   /* gLPF_MODE[6:5] gLPF_EN[4] aLPF_MODE[2:1] aLPF_EN[0] */
#define REG_CTRL7         0x08u   /* SyncSample[7] DRDY_DIS[5] gSN[4] gEN[1] aEN[0] */
#define REG_FIFO_CTRL     0x14u   /* FIFO_MODE[1:0] = 00 bypass */
#define REG_TIMESTAMP_L   0x30u   /* 0x30..0x32 TS, 0x33..0x34 TEMP, 0x35..0x3A A, 0x3B..0x40 G */
#define REG_RESET_FLAG    0x4Du   /* 0x80 after successful reset */
#define REG_RESET         0x60u
#define RESET_VALUE       0xB0u   /* Table 27 */

#define I2C_TIMEOUT_MS    3u      /* +length term in rd(); F4 HAL times the whole transfer */
#define TORN_RETRIES      3u
#define BURST_LEN         17u     /* 0x30..0x40 */

/* ---- bus helpers -------------------------------------------------------- */
static bool wr(Qmi8658 *d, uint8_t reg, uint8_t v)
{
    if (HAL_I2C_Mem_Write(d->hi2c, (uint16_t)(d->cfg.addr7 << 1), reg, 1u, &v, 1u,
                          I2C_TIMEOUT_MS) != HAL_OK) {
        d->i2c_errors++; d->last_hal_err = d->hi2c->ErrorCode; return false;
    }
    return true;
}

static bool rd(Qmi8658 *d, uint8_t reg, uint8_t *dst, uint16_t n)
{
    /* ~22.5 us/byte at 400 kHz; x2 margin. */
    uint32_t to = I2C_TIMEOUT_MS + ((uint32_t)n * 45u) / 1000u;
    if (HAL_I2C_Mem_Read(d->hi2c, (uint16_t)(d->cfg.addr7 << 1), reg, 1u, dst, n,
                         to) != HAL_OK) {
        d->i2c_errors++; d->last_hal_err = d->hi2c->ErrorCode; return false;
    }
    return true;
}

static inline int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint32_t le24(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

/* ---- public ------------------------------------------------------------- */
bool QMI8658_Init(Qmi8658 *d, I2C_HandleTypeDef *hi2c, const Qmi8658Config *cfg)
{
    static const float odr_tab[] = { 7174.4f, 3587.2f, 1793.6f, 896.8f, 448.4f,
                                     224.2f, 112.1f, 56.05f, 28.025f };
    memset(d, 0, sizeof(*d));
    d->hi2c = hi2c;
    d->cfg  = *cfg;
    d->a_lsb_per_g   = 16384.0f / (float)(1u << (uint32_t)cfg->afs);
    d->g_lsb_per_dps = 2048.0f  / (float)(1u << (uint32_t)cfg->gfs);
    d->odr_hz        = odr_tab[(uint32_t)cfg->odr];

    /* Identity, with retries: after an MCU reset mid-transfer the first
     * transaction can be mis-framed even after bus recovery (seen on bench). */
    uint8_t v = 0u;
    for (uint32_t t = 0u; t < 5u; t++) {
        v = 0u;
        if (rd(d, REG_WHO_AM_I, &v, 1u) && (v == QMI8658_WHO_AM_I_VAL)) { break; }
        HAL_Delay(2);
    }
    d->whoami = v;
    if (v != QMI8658_WHO_AM_I_VAL) { return false; }
    (void)rd(d, REG_REVISION_ID, &d->revision, 1u);

    (void)wr(d, REG_RESET, RESET_VALUE);             /* soft reset            */
    HAL_Delay(20);                                   /* t0 <= 15 ms           */
    (void)rd(d, REG_RESET_FLAG, &d->reset_flag, 1u); /* expect 0x80           */

    bool ok = true;
    ok = wr(d, REG_CTRL7, 0x00u) && ok;              /* sensors off while configuring */
    ok = wr(d, REG_CTRL1, 0x40u) && ok;              /* ADDR_AI=1, BE=0 (LE), INT pins hi-Z */
    ok = wr(d, REG_CTRL2, (uint8_t)(((uint32_t)cfg->afs << 4) | (uint32_t)cfg->odr)) && ok;
    ok = wr(d, REG_CTRL3, (uint8_t)(((uint32_t)cfg->gfs << 4) | (uint32_t)cfg->odr)) && ok;
    uint8_t c5 = 0u;
    if (cfg->lpf != QMI_LPF_OFF) {
        c5 = (uint8_t)(((uint32_t)cfg->lpf << 5) | 0x10u | ((uint32_t)cfg->lpf << 1) | 0x01u);
    }
    ok = wr(d, REG_CTRL5, c5) && ok;
    ok = wr(d, REG_FIFO_CTRL, 0x00u) && ok;          /* FIFO bypass           */
    ok = wr(d, REG_CTRL7, 0x03u) && ok;              /* aEN | gEN             */
    HAL_Delay(200);                                  /* gyro 150 ms + 3/ODR   */

    uint8_t rb[7] = {0};
    ok = rd(d, REG_CTRL1, rb, 7u) && ok;
    memcpy(d->ctrl_rb, rb, sizeof(rb));
    ok = ok && (rb[0] == 0x40u) && (rb[6] == 0x03u);
    d->i2c_errors = 0u;                              /* count runtime errors only */
    return ok;
}

bool QMI8658_ReadLatest(Qmi8658 *d, Qmi8658Raw *out, uint32_t *d_ts)
{
    uint8_t b[BURST_LEN];
    uint8_t t2[3];
    uint32_t ts = 0u;
    bool coherent = false;

    for (uint32_t k = 0u; k <= TORN_RETRIES; k++) {
        if (!rd(d, REG_TIMESTAMP_L, b, BURST_LEN)) { return false; }
        if (!rd(d, REG_TIMESTAMP_L, t2, 3u))        { return false; }
        ts = le24(b);
        if (ts == le24(t2)) { coherent = true; break; }
        d->torn_retries++;
    }
    if (!coherent) { d->torn_giveups++; return false; }

    out->ts   = ts;
    out->temp = le16(&b[3]);
    out->a[0] = le16(&b[5]);  out->a[1] = le16(&b[7]);  out->a[2] = le16(&b[9]);
    out->g[0] = le16(&b[11]); out->g[1] = le16(&b[13]); out->g[2] = le16(&b[15]);

    uint32_t dt = d->have_ts ? ((ts - d->ts_last) & 0xFFFFFFu) : 1u;
    if (dt == 0u) { d->stale++; }
    d->ts_last = ts; d->have_ts = true;
    d->reads++;
    if (d_ts != NULL) { *d_ts = dt; }
    return true;
}
