/* ============================================================================
 * qmi8658.h  —  QST QMI8658A 6-axis IMU driver (I2C, latest-sample mode)
 *
 * Board: Hyades Flight Controller V3 (STM32F411). The U7 footprint is the
 * ICM-42688-P, but the fitted part identifies as QMI8658A
 * (WHO_AM_I 0x00 = 0x05, REVISION_ID 0x01 = 0x7C) at 7-bit address 0x6B
 * (SA0 tied to GND). INT1/INT2 are not routed.
 *
 * Reference: QST QMI8658A Datasheet Rev A (Doc 13-52-25, 2022).
 *
 * WHY NOT THE FIFO (measured on this board, 2026-11-02):
 *   The FIFO stops filling while it is in read mode (FIFO_RD_MODE, set by the
 *   CTRL9 REQ_FIFO command until the host clears it). At 400 kHz I2C each
 *   12-byte frame costs ~0.27 ms to read, so draining 897 Hz data every 10 ms
 *   kept the FIFO in read mode ~25 % of the time and ~21 % of samples were
 *   lost (~700 of ~890 frames/s delivered, no overflow flag), for ~2.5 ms of
 *   blocking I2C per 10 ms frame.
 *
 * DESIGN: the sensor runs at QMI_ODR_224HZ with its LPF at 13.37 % ODR
 * (~30 Hz) acting as the anti-alias filter for a 100 Hz control frame. Each
 * frame the host reads the newest coherent sample from the data registers:
 * one 17-byte burst (0x30 TIMESTAMP .. 0x40 GZ_H), then a 3-byte TIMESTAMP
 * re-read. If the timestamp changed during the burst the sample may mix two
 * conversions and is re-read (bounded retries).
 *
 * Blocking HAL I2C with bounded timeouts (no I2C IRQ/DMA configured yet).
 * ============================================================================ */
#ifndef QMI8658_H_
#define QMI8658_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define QMI8658_ADDR_SA0_LOW     0x6Bu   /* SA0 = 0 (this board)            */
#define QMI8658_ADDR_SA0_HIGH    0x6Au   /* SA0 = 1 / floating              */
#define QMI8658_WHO_AM_I_VAL     0x05u

/* Full-scale codes (CTRL2[6:4], CTRL3[6:4]) */
typedef enum { QMI_AFS_2G = 0, QMI_AFS_4G, QMI_AFS_8G, QMI_AFS_16G } QmiAccelFs;
typedef enum { QMI_GFS_16 = 0, QMI_GFS_32, QMI_GFS_64, QMI_GFS_128,
               QMI_GFS_256, QMI_GFS_512, QMI_GFS_1024, QMI_GFS_2048 } QmiGyroFs;

/* ODR codes valid in 6DOF mode (accel ODR follows the gyro clock, +/-2 %). */
typedef enum {
    QMI_ODR_7174HZ = 0, QMI_ODR_3587HZ, QMI_ODR_1794HZ, QMI_ODR_897HZ,
    QMI_ODR_448HZ, QMI_ODR_224HZ, QMI_ODR_112HZ, QMI_ODR_56HZ, QMI_ODR_28HZ
} QmiOdr;

/* LPF bandwidth as a fraction of ODR (CTRL5 mode bits). */
typedef enum { QMI_LPF_2P66 = 0, QMI_LPF_3P63, QMI_LPF_5P39, QMI_LPF_13P37,
               QMI_LPF_OFF = 0xFF } QmiLpf;

typedef struct {
    uint8_t    addr7;     /* QMI8658_ADDR_SA0_LOW on this board        */
    QmiAccelFs afs;
    QmiGyroFs  gfs;
    QmiOdr     odr;
    QmiLpf     lpf;
} Qmi8658Config;

/* One coherent sample, sensor-native axes, raw counts. */
typedef struct {
    uint32_t ts;          /* 24-bit sample counter (+1 per ODR tick)   */
    int16_t  temp;        /* 256 LSB/degC                              */
    int16_t  a[3];
    int16_t  g[3];
} Qmi8658Raw;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    Qmi8658Config cfg;
    float    a_lsb_per_g;
    float    g_lsb_per_dps;
    float    odr_hz;          /* nominal */
    /* identity */
    uint8_t  whoami, revision, reset_flag;
    uint8_t  ctrl_rb[7];      /* CTRL1..CTRL7 read-back after init */
    /* health / diagnostics */
    uint32_t reads;           /* successful ReadLatest() calls            */
    uint32_t i2c_errors;
    uint32_t last_hal_err;    /* hi2c->ErrorCode at last failure          */
    uint32_t torn_retries;    /* TS changed mid-burst -> re-read          */
    uint32_t torn_giveups;    /* retries exhausted                        */
    uint32_t stale;           /* no new sample since previous call        */
    uint32_t ts_last;
    bool     have_ts;
} Qmi8658;

/* Identify, soft-reset, configure, enable sensors (FIFO bypassed).
 * Blocks ~220 ms (reset + gyro turn-on). False on ID mismatch / bus error. */
bool QMI8658_Init(Qmi8658 *d, I2C_HandleTypeDef *hi2c, const Qmi8658Config *cfg);

/* Read the newest coherent sample. Returns false on bus error or if no
 * coherent read was obtained within the retry bound. *d_ts receives the
 * number of ODR ticks since the previous successful call (0 = stale). */
bool QMI8658_ReadLatest(Qmi8658 *d, Qmi8658Raw *out, uint32_t *d_ts);

/* Unit conversion helpers (sensor frame). */
static inline float QMI8658_AccelG(const Qmi8658 *d, int16_t c)  { return (float)c / d->a_lsb_per_g; }
static inline float QMI8658_GyroDps(const Qmi8658 *d, int16_t c) { return (float)c / d->g_lsb_per_dps; }
static inline float QMI8658_TempC(int16_t c)                     { return (float)c / 256.0f; }

#endif /* QMI8658_H_ */
