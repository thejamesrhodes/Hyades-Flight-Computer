/* ============================================================================
 * imu.h  —  IMU array driver  (LSM303DLHC accel + mag, L3GD20 gyro, I2C1)
 *
 * Responsibilities:
 *   - initialise and self-test the three sensors,
 *   - read raw triads (bounded, timeout-protected I2C),
 *   - apply per-sensor calibration,
 *   - REMAP every triad into the common body frame (board.h),
 *   - report per-sensor health.
 *
 * Output of IMU_ReadBody() is the ONLY interface the estimator/control see:
 * everything downstream is in the body frame, in physical units.
 * ============================================================================ */
#ifndef IMU_H_
#define IMU_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- LSM303DLHC accelerometer (0x19) ---- */
#define LSM303_ACCEL_ADDR    0x19
#define LSM303_CTRL_REG1_A   0x20
#define LSM303_CTRL_REG4_A   0x23
#define LSM303_OUT_X_L_A     0x28

/* ---- LSM303DLHC magnetometer (0x1E) ---- */
#define LSM303_MAG_ADDR      0x1E
#define LSM303_CRA_REG_M     0x00
#define LSM303_CRB_REG_M     0x01
#define LSM303_MR_REG_M      0x02
#define LSM303_OUT_X_H_M     0x03
#define LSM303_IRA_REG_M     0x0A   /* identification, reads 0x48 ('H')        */

/* ---- L3GD20 gyroscope (0x69) ---- */
#define L3GD20_ADDR          0x69
#define L3GD20_WHO_AM_I      0x0F   /* 0xD4 (L3GD20) or 0xD7 (L3GD20H)         */
#define L3GD20_CTRL_REG1     0x20
#define L3GD20_CTRL_REG4     0x23
#define L3GD20_OUT_X_L       0x28

#define IMU_CALIB_SAMPLES    256u

/* Calibration parameters (persist between reads; populated by IMU_Calibrate). */
typedef struct { float bias[3]; } GyroCal;    /* dps,    sensor frame          */
typedef struct { float bias[3]; } AccelCal;   /* m/s^2,  BODY frame            */
typedef struct {                               /* Gauss,  sensor frame          */
    float hard[3];    /* hard-iron offset  */
    float scale[3];   /* soft-iron scale   */
} MagCal;

/* One synchronous sample, fully conditioned, in the BODY frame. */
typedef struct {
    float accel[3];   /* m/s^2,  calibrated, body frame                        */
    float gyro[3];    /* dps, body; static boot bias removed, IEKF tracks residual */
    float mag[3];     /* Gauss,  calibrated, body frame                        */
    bool  accel_ok;   /* I2C transaction + range sane                          */
    bool  gyro_ok;
    bool  mag_ok;
} ImuSample;

/* Init + power-on self-test. Returns false if any sensor fails identification. */
bool IMU_Init(I2C_HandleTypeDef *hi2c);
bool IMU_SelfTest(I2C_HandleTypeDef *hi2c);

/* Static calibration — airframe must be STATIONARY (any orientation).
 * Estimates gyro zero-rate bias (sensor frame). Accel bias is zeroed: it is
 * not separable from tilt in a single pose (attitude is seeded from accel). */
void IMU_Calibrate(I2C_HandleTypeDef *hi2c, GyroCal *gc, AccelCal *ac);

/* Interactive magnetometer hard/soft-iron calibration (rotate through all
 * orientations). Optional; gated by CFG_ENABLE_MAG_CAL. */
void IMU_MagCalibrate(I2C_HandleTypeDef *hi2c, MagCal *mc);
void MagCal_Identity(MagCal *mc);

/* Read all three sensors once, condition, remap to body frame. Never blocks
 * unbounded: each I2C transfer carries a hard timeout; on failure the relevant
 * *_ok flag is cleared and that triad is left zeroed (caller skips its update). */
void IMU_ReadBody(I2C_HandleTypeDef *hi2c,
                  const GyroCal *gc, const AccelCal *ac, const MagCal *mc,
                  ImuSample *out);

#endif /* IMU_H_ */
