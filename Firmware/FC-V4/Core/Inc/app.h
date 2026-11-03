/* ============================================================================
 * app.h  —  FC-V4 application: boot sequence, cyclic executive, fault arbiter
 *
 * Layering (h743_flight_architecture §2.1). Calls go downward only.
 *   L4 mission  Core/hyades/mission  phase FSM, FDIR engine        (pure)
 *   L3 GNC      Core/hyades/gnc      IEKF, SO(3)+INDI control       (pure)
 *   L2 services Core/hyades/svc      NMEA, log records, CRC         (pure)
 *               Core/Src             console, flash log, fw update, telemetry
 *   L1 platform Core/Src             app (sequencing), sched, wdg, usbio, leds
 *   L0 drivers  Core/Src             qmi8658/imu, pwm, gps (UART), HAL
 * Core/hyades has no HAL dependency and is unit-tested on the host (Tests/);
 * it is the part that moves to the H743 unchanged.
 * ============================================================================ */
#ifndef APP_H_
#define APP_H_

#include "main.h"

/* Never returns. Called from main() after the CubeMX peripheral init. */
void App_Run(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) __attribute__((noreturn));

#endif /* APP_H_ */
