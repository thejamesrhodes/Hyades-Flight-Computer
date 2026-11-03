/* ============================================================================
 * gps.h  —  ATGM336H (AT6558) GNSS receiver, receive-only, either UART port
 *
 * The F411 board has two UART headers: port 1 = USART1 (RX PA10, TX PA15) and
 * port 2 = USART2 (RX PA3, TX PA2). The original V4 firmware ran the GPS on
 * port 2; it is easy to plug the module into either, so both RX lines are
 * enabled and feed one ring/parser — whichever port the module is on works.
 * At boot GPS_Probe() classifies every line (pulled-up/down reads -> floating,
 * held high, held low), counts edges and measures the shortest pulse to infer
 * the baud rate, so the boot banner says exactly what is (not) connected.
 * If a port then receives bytes that never parse as NMEA, the receiver steps
 * through the standard rates until they do (GPS_Poll), so a wrong guess or a
 * module set to another rate still ends up working.
 *
 * The RX ISRs only copy bytes into a ring (std B3); parsing (Core/hyades/svc/
 * nmea.c) runs from the main loop, bounded per call. Position is status/log
 * only — not fused into the estimator.
 * ============================================================================ */
#ifndef GPS_H_
#define GPS_H_

#include "main.h"
#include "nmea.h"
#include <stdint.h>
#include <stdbool.h>

#define GPS_NPINS 4u          /* PA10 rx1, PA15 tx1, PA3 rx2, PA2 tx2 */

typedef enum { LINE_FLOAT = 0, LINE_HIGH, LINE_LOW } LineLevel;

typedef struct {
    uint8_t  level[GPS_NPINS];      /* LineLevel at rest (pull-up vs pull-down) */
    uint32_t edges[GPS_NPINS];      /* transitions in the probe window          */
    uint32_t baud[GPS_NPINS];       /* snapped from the shortest pulse, 0 = ?   */
    uint8_t  port;                  /* 1 or 2: RX line with traffic, 0 = none   */
    bool     crossed;               /* traffic only on a TX pin                 */
} GpsProbe;

/* Blocking (window_ms, >= 1100 to span a 1 Hz burst): call at boot, before the
 * watchdog starts. Then enables both ports' receivers (probed baud, else 9600). */
void            GPS_Init(uint32_t probe_window_ms);
const GpsProbe *GPS_ProbeResult(void);
const char     *GPS_PinName(uint32_t i);
const char     *GPS_LevelName(uint8_t lvl);
const char     *GPS_Advice(void);           /* one-line diagnosis of the probe */

void           GPS_Poll(uint32_t max_bytes);
const GpsData *GPS_Get(void);
uint32_t       GPS_Overruns(void);
uint32_t       GPS_PortBytes(uint32_t port);      /* 1 or 2 */
uint32_t       GPS_PortBaud(uint32_t port);
uint32_t       GPS_Retunes(void);                 /* runtime baud-hunt steps taken */
bool           GPS_Alive(uint32_t timeout_ms);
bool           GPS_HasFix(uint32_t timeout_ms);

/* Last few raw NMEA lines (oldest first), for 'gps raw'. */
#define GPS_RAW_LINES 4u
uint32_t       GPS_RawLines(const char *out[GPS_RAW_LINES]);

#endif /* GPS_H_ */
