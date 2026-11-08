/* ============================================================================
 * imu.h  —  Body-frame IMU samples from the on-board QMI8658A (I2C1)
 *
 * The ONLY interface the estimator/controller see: body frame (board.h
 * REMAP_QMI), physical units, per-sample health. No magnetometer on FC-V4.
 *   accel : m/s^2 specific force, no bias removal (see IMU_CalibrateGyro)
 *   gyro  : dps, boot zero-rate bias removed (the IEKF tracks the residual)
 * Sensor: +/-16 g, +/-2048 dps, 448.4 Hz ODR (ADR-004; 224.2 Hz until 7 Nov), LPF 13.37 % ODR (~60 Hz AAF for
 * the 100 Hz frame). One coherent latest-sample read per call: ~0.68 ms typ,
 * 1.25 ms max blocking on 400 kHz I2C (bench, 2026-11-02). Blocking HAL I2C
 * with a length-scaled timeout is a documented deviation from std C6 on this
 * board; the H743 build moves the IMU to SPI + DMA.
 * ============================================================================ */
#ifndef IMU_H_
#define IMU_H_

#include "main.h"
#include "qmi8658.h"
#include <stdint.h>
#include <stdbool.h>

#define IMU_CALIB_SAMPLES 256u

typedef struct { float bias[3]; } GyroCal;     /* dps, sensor frame */

typedef struct {
    float accel[3];      /* m/s^2, body                   */
    float gyro[3];       /* dps, body, boot bias removed  */
    bool  ok;            /* coherent read succeeded       */
} ImuSample;

/* Bus recovery, identify, configure, and a plausibility self-test
 * (|a| in 0.5..1.5 g at rest). Returns false on any failure. */
bool IMU_Init(I2C_HandleTypeDef *hi2c);

/* Average IMU_CALIB_SAMPLES fresh gyro samples (~1.3 s, blocking, boot only).
 * Airframe must be still; any orientation. */
void IMU_CalibrateGyro(GyroCal *gc);

void IMU_ReadBody(const GyroCal *gc, ImuSample *out);

/* Raw sensor counts (sensor frame), for the bench noise capture only.
 * *dts = ODR ticks since the previous read (0 = no new sample). */
bool IMU_ReadRaw(Qmi8658Raw *r, uint32_t *dts);

/* Driver diagnostics (read counts, I2C errors, torn reads ...). */
const Qmi8658 *IMU_Driver(void);

#endif /* IMU_H_ */
