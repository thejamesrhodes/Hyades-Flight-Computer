/* ============================================================================
 * gps.h  —  ATGM336H (AT6558) GNSS receiver on USART1, receive-only
 *
 * Wiring (FC-V3/V4 board): module TX -> PA10 (USART1_RX), 9600 8N1, 1 Hz NMEA.
 * PA15 (USART1_TX) is unused and held as a pulled-up input.
 * The RX ISR only copies bytes into a ring (std B3); parsing (Core/hyades/svc/
 * nmea.c) runs from the main loop, bounded per call. Position is status/log
 * only — not fused into the estimator.
 * ============================================================================ */
#ifndef GPS_H_
#define GPS_H_

#include "main.h"
#include "nmea.h"
#include <stdint.h>
#include <stdbool.h>

void           GPS_Init(UART_HandleTypeDef *huart);
void           GPS_Poll(uint32_t max_bytes);
const GpsData *GPS_Get(void);
uint32_t       GPS_Overruns(void);
bool           GPS_Alive(uint32_t timeout_ms);
bool           GPS_HasFix(uint32_t timeout_ms);

/* Boot wiring check (blocking): logic edges seen on PA10 and PA15 over
 * window_ms (use >= 1100 ms to span one 1 Hz burst). Activity only on PA15
 * means module TX/RX are crossed; none on either means no signal at all. */
void           GPS_LineActivity(uint32_t window_ms, uint32_t *edges_pa10, uint32_t *edges_pa15);

#endif /* GPS_H_ */
