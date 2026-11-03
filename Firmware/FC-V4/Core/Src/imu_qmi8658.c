/* ============================================================================
 * imu_qmi8658.c  —  imu.h backend for the on-board QMI8658A (V3 board, U7)
 *
 * Drop-in replacement for imu.c: same API (imu.h), selected with
 * IMU_BACKEND_QMI8658 = 1 in board.h. main.c needs no change (boot log only).
 *
 *   accel : m/s^2, body frame (REMAP_QMI), no bias removal (see IMU_Calibrate)
 *   gyro  : dps,   body frame, zero-rate bias removed in the sensor frame
 *   mag   : none on this board -> mag_ok = false, triad zero. IEKF mag updates
 *           are therefore skipped; yaw is unobservable (gyro-integrated only).
 *
 * Sensor config: +/-16 g, +/-2048 dps, 224.2 Hz ODR, LPF 13.37 % ODR (~30 Hz
 * anti-alias for the 100 Hz frame). One coherent latest-sample read per call,
 * ~0.68 ms typ / 1.25 ms max blocking on 400 kHz I2C (bench, 2026-11-02).
 *
 * BOARD ASSUMPTION: hi2c is I2C1 on PB6 (SCL) / PB7 (SDA) — used by the bus
 * recovery in IMU_Init().
 * ============================================================================ */
#include "imu.h"
#include "board.h"
#if IMU_BACKEND_QMI8658
#include "qmi8658.h"
#include <string.h>
#include <math.h>

static Qmi8658 s_qmi;

/* Exposed for telemetry/debug (read with the debugger). */
volatile uint32_t imu_qmi_last_dts = 0u;   /* ODR ticks since previous read */

/* ---- I2C1 bus recovery (PB6 = SCL, PB7 = SDA) ---------------------------
 * If the MCU resets mid-read the slave can hold SDA low / be mid-byte (seen
 * on the bench: first WHO_AM_I after a debugger reset read 0x73). Clock SCL
 * 9x as open-drain GPIO, issue STOP, RCC-reset the peripheral, re-init. */
static void short_delay(void) { for (volatile uint32_t i = 0u; i < 120u; i++) { } } /* ~5 us @96 MHz */
static void i2c1_bus_recover(I2C_HandleTypeDef *hi2c)
{
    (void)HAL_I2C_DeInit(hi2c);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gi = {0};
    gi.Pin = GPIO_PIN_6 | GPIO_PIN_7; gi.Mode = GPIO_MODE_OUTPUT_OD;
    gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_Init(GPIOB, &gi);
    short_delay();
    for (uint32_t k = 0u; k < 9u; k++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); short_delay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   short_delay();
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); short_delay();   /* STOP */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   short_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   short_delay();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    (void)HAL_I2C_Init(hi2c);      /* MspInit restores AF4 open-drain */
}

bool IMU_Init(I2C_HandleTypeDef *hi2c)
{
    hi2c->Init.ClockSpeed = 400000u;          /* CubeMX default here is 100 kHz */
    hi2c->Init.DutyCycle  = I2C_DUTYCYCLE_2;
    i2c1_bus_recover(hi2c);

    const Qmi8658Config cfg = { QMI8658_ADDR_SA0_LOW, QMI_AFS_16G, QMI_GFS_2048,
                                QMI_ODR_224HZ, QMI_LPF_13P37 };
    if (!QMI8658_Init(&s_qmi, hi2c, &cfg)) { return false; }
    return IMU_SelfTest(hi2c);
}

bool IMU_SelfTest(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
    /* Identity + one coherent read with a physically plausible |a|. */
    if (s_qmi.whoami != QMI8658_WHO_AM_I_VAL) { return false; }
    Qmi8658Raw r; uint32_t dts = 0u;
    bool ok = false;
    for (uint32_t i = 0u; (i < 10u) && !ok; i++) {
        if (QMI8658_ReadLatest(&s_qmi, &r, &dts)) {
            float ax = QMI8658_AccelG(&s_qmi, r.a[0]);
            float ay = QMI8658_AccelG(&s_qmi, r.a[1]);
            float az = QMI8658_AccelG(&s_qmi, r.a[2]);
            float n = sqrtf(ax*ax + ay*ay + az*az);
            ok = (n > 0.5f) && (n < 1.5f);           /* stationary at boot */
        }
        HAL_Delay(5);
    }
    return ok;
}

/* Raw read in sensor frame, physical units: accel m/s^2, gyro dps. */
static bool read_raw(float a[3], float g[3])
{
    Qmi8658Raw r; uint32_t dts = 0u;
    if (!QMI8658_ReadLatest(&s_qmi, &r, &dts)) { return false; }
    imu_qmi_last_dts = dts;
    for (uint32_t k = 0u; k < 3u; k++) {
        a[k] = QMI8658_AccelG(&s_qmi, r.a[k]) * BOARD_G_MPS2;
        g[k] = QMI8658_GyroDps(&s_qmi, r.g[k]);
    }
    return true;
}

void IMU_ReadBody(I2C_HandleTypeDef *hi2c,
                  const GyroCal *gc, const AccelCal *ac, const MagCal *mc,
                  ImuSample *out)
{
    (void)hi2c; (void)mc;
    float a[3], g[3];
    if (read_raw(a, g)) {
        float bx, by, bz;
        REMAP_QMI(a[0], a[1], a[2], bx, by, bz);
        out->accel[0] = bx - ac->bias[0];
        out->accel[1] = by - ac->bias[1];
        out->accel[2] = bz - ac->bias[2];
        g[0] -= gc->bias[0]; g[1] -= gc->bias[1]; g[2] -= gc->bias[2];
        REMAP_QMI(g[0], g[1], g[2], bx, by, bz);
        out->gyro[0] = bx; out->gyro[1] = by; out->gyro[2] = bz;
        out->accel_ok = true;
        out->gyro_ok  = true;
    } else {
        memset(out->accel, 0, sizeof(out->accel));
        memset(out->gyro,  0, sizeof(out->gyro));
        out->accel_ok = false;
        out->gyro_ok  = false;
    }
    memset(out->mag, 0, sizeof(out->mag));
    out->mag_ok = false;                      /* no magnetometer on this board */
}

void IMU_Calibrate(I2C_HandleTypeDef *hi2c, GyroCal *gc, AccelCal *ac)
{
    (void)hi2c;
    float sg[3] = {0};
    uint32_t n = 0u;
    gc->bias[0] = gc->bias[1] = gc->bias[2] = 0.0f;
    /* Accel bias is NOT estimated from one static pose: it is inseparable from
     * tilt there, and absorbing it would force the seed to read "nose-up"
     * whatever the real attitude. Left at zero (QMI8658A offset is tens of mg,
     * ~1-3 deg); set from a multi-pose calibration if ever needed. */
    ac->bias[0] = ac->bias[1] = ac->bias[2] = 0.0f;

    for (uint32_t i = 0u; i < IMU_CALIB_SAMPLES; i++) {
        float a[3], g[3];
        if (read_raw(a, g) && (imu_qmi_last_dts != 0u)) {     /* fresh samples only */
            sg[0] += g[0]; sg[1] += g[1]; sg[2] += g[2];
            n++;
        }
        HAL_Delay(5);                         /* > 1/224 Hz -> new sample each time */
    }
    if (n > 0u) {
        gc->bias[0] = sg[0]/(float)n; gc->bias[1] = sg[1]/(float)n; gc->bias[2] = sg[2]/(float)n;
    }
}

void MagCal_Identity(MagCal *mc)
{
    mc->hard[0] = mc->hard[1] = mc->hard[2] = 0.0f;
    mc->scale[0] = mc->scale[1] = mc->scale[2] = 1.0f;
}

void IMU_MagCalibrate(I2C_HandleTypeDef *hi2c, MagCal *mc)
{
    (void)hi2c;
    MagCal_Identity(mc);                      /* no magnetometer */
}

#endif /* IMU_BACKEND_QMI8658 */
