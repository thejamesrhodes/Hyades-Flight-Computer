/* ============================================================================
 * imu.c  —  IMU array driver
 *
 * Sensors (I2C1, 400 kHz):
 *   LSM303DLHC accel  (ST DocID018771) : 12-bit left-justified, 1 mg/LSB @ +/-2g
 *   LSM303DLHC mag    (ST DocID018771) : XY 1100 LSB/Ga, Z 980 LSB/Ga @ +/-1.3Ga
 *                                        readout order X,Z,Y (datasheet)
 *   L3GD20 gyro       (ST DM00031089)  : 8.75 mdps/LSB @ +/-245 dps
 *
 * I2C policy (deliberate, documented deviation from the no-blocking rule):
 *   These are I2C-only parts; there is no SPI path on this board. Each transfer
 *   uses a bounded HAL timeout (I2C_TIMEOUT_MS). At 400 kHz three 6-byte reads
 *   cost ~0.6 ms, well under the 10 ms frame, and the timeout makes the WCET
 *   finite. DMA/IT is the upgrade path but the F4 I2C-v1 errata make it poor
 *   value at this rate. See README.
 * ============================================================================ */
#include "imu.h"
#include "board.h"
#if !IMU_BACKEND_QMI8658   /* legacy LSM303/L3GD20 backend */
#include <string.h>
#include <math.h>

#define I2C_TIMEOUT_MS   5u

/* ----------------------------------------------------------------------------
 * Low-level I2C helpers (all bounded)
 * -------------------------------------------------------------------------- */
static HAL_StatusTypeDef i2c_w(I2C_HandleTypeDef *hi2c, uint8_t addr,
                               uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return HAL_I2C_Master_Transmit(hi2c, (uint16_t)(addr << 1), b, 2u, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef i2c_r(I2C_HandleTypeDef *hi2c, uint8_t addr,
                               uint8_t reg, uint8_t *dst, uint8_t len)
{
    HAL_StatusTypeDef s;
    reg |= 0x80u;   /* auto-increment for multi-byte reads */
    s = HAL_I2C_Master_Transmit(hi2c, (uint16_t)(addr << 1), &reg, 1u, I2C_TIMEOUT_MS);
    if (s != HAL_OK) { return s; }
    return HAL_I2C_Master_Receive(hi2c, (uint16_t)(addr << 1), dst, len, I2C_TIMEOUT_MS);
}

/* ----------------------------------------------------------------------------
 * Raw reads (sensor-native frame, physical units, no calibration)
 * -------------------------------------------------------------------------- */
static bool accel_read_raw(I2C_HandleTypeDef *hi2c, float v[3])
{
    uint8_t b[6];
    if (i2c_r(hi2c, LSM303_ACCEL_ADDR, LSM303_OUT_X_L_A, b, 6u) != HAL_OK) { return false; }
    const float k = 9.80665e-3f;   /* 1 mg/LSB -> m/s^2, 12-bit left-justified  */
    v[0] = (float)((int16_t)((b[1] << 8) | b[0]) >> 4) * k;
    v[1] = (float)((int16_t)((b[3] << 8) | b[2]) >> 4) * k;
    v[2] = (float)((int16_t)((b[5] << 8) | b[4]) >> 4) * k;
    return true;
}

static bool gyro_read_raw(I2C_HandleTypeDef *hi2c, float v[3])
{
    uint8_t b[6];
    if (i2c_r(hi2c, L3GD20_ADDR, L3GD20_OUT_X_L, b, 6u) != HAL_OK) { return false; }
    const float k = 8.75e-3f;      /* mdps/LSB -> dps @ +/-245 dps               */
    v[0] = (float)((int16_t)((b[1] << 8) | b[0])) * k;
    v[1] = (float)((int16_t)((b[3] << 8) | b[2])) * k;
    v[2] = (float)((int16_t)((b[5] << 8) | b[4])) * k;
    return true;
}

/* Mag returns Gauss in sensor frame, BEFORE hard/soft-iron (raw calibrated by
 * caller). Note LSM303 mag readout order is X(H,L) Z(H,L) Y(H,L). */
static bool mag_read_raw(I2C_HandleTypeDef *hi2c, float v[3])
{
    uint8_t b[6];
    uint8_t reg = LSM303_OUT_X_H_M;
    if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)(LSM303_MAG_ADDR << 1), &reg, 1u,
                                I2C_TIMEOUT_MS) != HAL_OK) { return false; }
    if (HAL_I2C_Master_Receive(hi2c, (uint16_t)(LSM303_MAG_ADDR << 1), b, 6u,
                               I2C_TIMEOUT_MS) != HAL_OK) { return false; }
    v[0] = (float)((int16_t)((b[0] << 8) | b[1])) / 1100.0f;  /* X */
    v[2] = (float)((int16_t)((b[2] << 8) | b[3])) /  980.0f;  /* Z */
    v[1] = (float)((int16_t)((b[4] << 8) | b[5])) / 1100.0f;  /* Y */
    return true;
}

/* ----------------------------------------------------------------------------
 * Init + self-test
 * -------------------------------------------------------------------------- */
bool IMU_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef s = HAL_OK;

    /* Accel: 100 Hz, all axes, BDU + high-res. */
    s |= i2c_w(hi2c, LSM303_ACCEL_ADDR, LSM303_CTRL_REG1_A, 0x57u); /* 100Hz XYZ */
    s |= i2c_w(hi2c, LSM303_ACCEL_ADDR, LSM303_CTRL_REG4_A, 0x88u); /* BDU, HR    */

    /* Gyro: 190 Hz ODR / 50 Hz cut, all axes; BDU, +/-245 dps. */
    s |= i2c_w(hi2c, L3GD20_ADDR, L3GD20_CTRL_REG1, 0x5Fu);         /* 190Hz XYZ  */
    s |= i2c_w(hi2c, L3GD20_ADDR, L3GD20_CTRL_REG4, 0x80u);         /* BDU, 245   */

    /* Mag: 220 Hz, +/-1.3 Ga, continuous. */
    s |= i2c_w(hi2c, LSM303_MAG_ADDR, LSM303_CRA_REG_M, 0x1Cu);
    s |= i2c_w(hi2c, LSM303_MAG_ADDR, LSM303_CRB_REG_M, 0x20u);
    s |= i2c_w(hi2c, LSM303_MAG_ADDR, LSM303_MR_REG_M,  0x00u);

    HAL_Delay(50);   /* gyro start-up + first conversions (boot-time, allowed)  */
    return (s == HAL_OK) && IMU_SelfTest(hi2c);
}

bool IMU_SelfTest(I2C_HandleTypeDef *hi2c)
{
    uint8_t id = 0;
    bool ok = true;

    /* L3GD20 WHO_AM_I. */
    if (i2c_r(hi2c, L3GD20_ADDR, L3GD20_WHO_AM_I, &id, 1u) != HAL_OK) { ok = false; }
    else if (id != 0xD4u && id != 0xD7u && id != 0xD3u) { ok = false; }

    /* LSM303 mag identification register A reads ASCII 'H' (0x48). */
    if (i2c_r(hi2c, LSM303_MAG_ADDR, LSM303_IRA_REG_M, &id, 1u) != HAL_OK) { ok = false; }
    else if (id != 0x48u) { ok = false; }

    /* LSM303 accel has no WHO_AM_I; verify CTRL_REG1 read-back instead. */
    if (i2c_r(hi2c, LSM303_ACCEL_ADDR, LSM303_CTRL_REG1_A, &id, 1u) != HAL_OK) { ok = false; }
    else if (id != 0x57u) { ok = false; }

    return ok;
}

/* ----------------------------------------------------------------------------
 * Body-frame conditioned read
 * -------------------------------------------------------------------------- */
void IMU_ReadBody(I2C_HandleTypeDef *hi2c,
                  const GyroCal *gc, const AccelCal *ac, const MagCal *mc,
                  ImuSample *out)
{
    float a[3], g[3], m[3];

    /* ----- accelerometer ----- */
    if (accel_read_raw(hi2c, a)) {
        float bx, by, bz;
        REMAP_ACCEL(a[0], a[1], a[2], bx, by, bz);
        out->accel[0] = bx - ac->bias[0];
        out->accel[1] = by - ac->bias[1];
        out->accel[2] = bz - ac->bias[2];
        out->accel_ok = true;
    } else {
        out->accel[0] = out->accel[1] = out->accel[2] = 0.0f;
        out->accel_ok = false;
    }

    /* ----- gyroscope (bias left for the MEKF state) ----- */
    if (gyro_read_raw(hi2c, g)) {
        float bx, by, bz;
        /* zero-rate bias is in the sensor frame, remove before remap */
        g[0] -= gc->bias[0]; g[1] -= gc->bias[1]; g[2] -= gc->bias[2];
        REMAP_GYRO(g[0], g[1], g[2], bx, by, bz);
        out->gyro[0] = bx; out->gyro[1] = by; out->gyro[2] = bz;
        out->gyro_ok = true;
    } else {
        out->gyro[0] = out->gyro[1] = out->gyro[2] = 0.0f;
        out->gyro_ok = false;
    }

    /* ----- magnetometer (hard/soft-iron in sensor frame, then remap) ----- */
    if (mag_read_raw(hi2c, m)) {
        float bx, by, bz;
        m[0] = (m[0] - mc->hard[0]) * mc->scale[0];
        m[1] = (m[1] - mc->hard[1]) * mc->scale[1];
        m[2] = (m[2] - mc->hard[2]) * mc->scale[2];
        REMAP_MAG(m[0], m[1], m[2], bx, by, bz);
        out->mag[0] = bx; out->mag[1] = by; out->mag[2] = bz;
        out->mag_ok = true;
    } else {
        out->mag[0] = out->mag[1] = out->mag[2] = 0.0f;
        out->mag_ok = false;
    }
}

/* ----------------------------------------------------------------------------
 * Calibration
 * -------------------------------------------------------------------------- */
void IMU_Calibrate(I2C_HandleTypeDef *hi2c, GyroCal *gc, AccelCal *ac)
{
    float sg[3] = {0}, sa[3] = {0};
    uint32_t ng = 0u, na = 0u;

    gc->bias[0] = gc->bias[1] = gc->bias[2] = 0.0f;
    ac->bias[0] = ac->bias[1] = ac->bias[2] = 0.0f;

    for (uint32_t i = 0u; i < IMU_CALIB_SAMPLES; i++) {
        float a[3], g[3];
        if (gyro_read_raw(hi2c, g)) { sg[0]+=g[0]; sg[1]+=g[1]; sg[2]+=g[2]; ng++; }
        if (accel_read_raw(hi2c, a)) {
            float bx, by, bz;
            REMAP_ACCEL(a[0], a[1], a[2], bx, by, bz);
            sa[0]+=bx; sa[1]+=by; sa[2]+=bz; na++;
        }
        HAL_Delay(4);
    }

    if (ng > 0u) {
        gc->bias[0] = sg[0]/(float)ng;
        gc->bias[1] = sg[1]/(float)ng;
        gc->bias[2] = sg[2]/(float)ng;
    }
    if (na > 0u) {
        /* expected body specific force at rest (nose-up) is (0,0,+g) */
        ac->bias[0] = sa[0]/(float)na - BOARD_GRAV_REF_E;
        ac->bias[1] = sa[1]/(float)na - BOARD_GRAV_REF_N;
        ac->bias[2] = sa[2]/(float)na - BOARD_GRAV_REF_U;
    }
}

void MagCal_Identity(MagCal *mc)
{
    mc->hard[0] = mc->hard[1] = mc->hard[2] = 0.0f;
    mc->scale[0] = mc->scale[1] = mc->scale[2] = 1.0f;
}

void IMU_MagCalibrate(I2C_HandleTypeDef *hi2c, MagCal *mc)
{
    float mn[3] = { 1e9f, 1e9f, 1e9f }, mx[3] = { -1e9f, -1e9f, -1e9f };
    MagCal_Identity(mc);

    for (uint32_t i = 0u; i < IMU_CALIB_SAMPLES * 4u; i++) {
        float m[3];
        if (mag_read_raw(hi2c, m)) {
            for (int k = 0; k < 3; k++) {
                if (m[k] < mn[k]) { mn[k] = m[k]; }
                if (m[k] > mx[k]) { mx[k] = m[k]; }
            }
        }
        HAL_Delay(10);
    }

    float r[3], avg = 0.0f;
    for (int k = 0; k < 3; k++) { r[k] = 0.5f * (mx[k] - mn[k]); avg += r[k]; }
    avg /= 3.0f;
    for (int k = 0; k < 3; k++) {
        mc->hard[k]  = 0.5f * (mx[k] + mn[k]);
        mc->scale[k] = (r[k] > 0.01f) ? (avg / r[k]) : 1.0f;
    }
}

#endif /* !IMU_BACKEND_QMI8658 */
