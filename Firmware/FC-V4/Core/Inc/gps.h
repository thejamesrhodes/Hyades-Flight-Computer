/* ============================================================================
 * gps.h  —  ATGM336H (AT6558) GNSS receiver, receive-only, USART2
 *
 * Wiring (FC-V3/V4 board, UART header 2): module TX -> PA3 (USART2_RX),
 * module RX <- PA2 (USART2_TX, unused, held as a pulled-up input), 9600 8N1,
 * 1 Hz NMEA. (Header 1, USART1 PA10/PA15, is unused.)
 *
 * At boot GPS_Init() probes PA3/PA2 — pulled-up vs pulled-down read (floating
 * / held high / held low), edge count and the bit time over the window — so
 * the banner says exactly what is (not) connected. Then USART2 RX runs into a
 * ring from its ISR (std B3); parsing (Core/hyades/svc/nmea.c) runs from the
 * main loop, bounded per call. Position is status/log only — not fused into
 * the estimator.
 * ============================================================================ */
#ifndef GPS_H_
#define GPS_H_

#include "main.h"
#include "nmea.h"
#include <stdint.h>
#include <stdbool.h>

#define GPS_BAUD 9600u

typedef enum { LINE_FLOAT = 0, LINE_HIGH, LINE_LOW } LineLevel;

typedef struct {
    uint8_t  rx_level, tx_level;    /* LineLevel at rest                         */
    uint32_t rx_edges, tx_edges;    /* transitions in the probe window           */
    uint32_t rx_baud;               /* from the bit time, 0 = not recognised     */
} GpsProbe;

/* Blocking (window_ms, >= 1100 to span a 1 Hz burst): call at boot, before the
 * watchdog starts. Then starts the USART2 receiver at GPS_BAUD. */
void            GPS_Init(uint32_t probe_window_ms);
const GpsProbe *GPS_ProbeResult(void);
const char     *GPS_LevelName(uint8_t lvl);
const char     *GPS_Advice(void);           /* one-line diagnosis of the probe */

void           GPS_Poll(uint32_t max_bytes);
const GpsData *GPS_Get(void);
uint32_t       GPS_Overruns(void);
bool           GPS_Alive(uint32_t timeout_ms);
bool           GPS_HasFix(uint32_t timeout_ms);

/* Last few raw NMEA lines (oldest first), for 'gps raw'. */
#define GPS_RAW_LINES 4u
uint32_t       GPS_RawLines(const char *out[GPS_RAW_LINES]);

#endif /* GPS_H_ */
