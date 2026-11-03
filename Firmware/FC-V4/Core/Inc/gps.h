/* ============================================================================
 * gps.h  —  NMEA-0183 GNSS receiver (ATGM336H / AT6558) on USART1
 *
 * Wiring (V3/V4 board): module TX -> PA10 (USART1_RX). Receive-only: PA15
 * (USART1_TX) is left as a pulled-up input. Default ATGM336H output is
 * 9600 8N1, 1 Hz, GN/GP/BD talkers, sent with or without a fix.
 *
 * RX is interrupt-driven into a ring buffer (ISR: copy byte, exit). Parsing
 * runs from the main loop via GPS_Poll() — bounded work per call, no blocking.
 * Parsed sentences: GGA (fix quality, sats, HDOP, MSL alt, lat/lon, UTC) and
 * RMC (validity, ground speed, course). All other sentences are counted and
 * ignored. Any talker ID is accepted ($GNGGA, $GPGGA, $BDGGA ...).
 * ============================================================================ */
#ifndef GPS_H_
#define GPS_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* GGA */
    uint8_t  fix_quality;     /* 0 none, 1 GPS, 2 DGPS, ... (GGA field 6)  */
    uint8_t  sats;            /* satellites used                           */
    float    hdop;
    float    alt_msl_m;
    double   lat_deg, lon_deg;/* +N / +E                                   */
    uint32_t utc_hhmmss;      /* integer hhmmss                            */
    /* RMC */
    bool     rmc_valid;       /* 'A' = valid                               */
    float    speed_mps;
    float    course_deg;
    /* bookkeeping */
    uint32_t last_gga_ms;     /* HAL_GetTick() at last GGA                 */
    uint32_t last_rx_ms;      /* HAL_GetTick() at last valid sentence      */
    /* diagnostics */
    uint32_t bytes;           /* raw bytes received                        */
    uint32_t sentences;       /* checksum-valid sentences                  */
    uint32_t gga, rmc;
    uint32_t cksum_err;
    uint32_t overrun;         /* ring-buffer or UART overrun               */
} GpsData;

extern GpsData gps;            /* main-loop only (written by GPS_Poll)    */

/* Start interrupt-driven reception on an initialised UART (USART1). */
void GPS_Init(UART_HandleTypeDef *huart);

/* Drain the ring buffer and parse complete sentences. Call every frame.
 * Bounded: processes at most max_bytes per call. */
void GPS_Poll(uint32_t max_bytes);

/* Boot diagnostic (blocking): count logic edges on PA10 and PA15 over
 * window_ms. Use >= 1100 ms to span one 1 Hz NMEA burst. Activity on PA15
 * only means the module's TX is wired to the wrong pin (TX/RX crossed). */
void GPS_LineActivity(uint32_t window_ms, uint32_t *edges_pa10, uint32_t *edges_pa15);

/* true if a checksum-valid sentence arrived in the last timeout_ms */
bool GPS_Alive(uint32_t timeout_ms);
/* true if the last GGA (within timeout_ms) reported a position fix */
bool GPS_HasFix(uint32_t timeout_ms);

#endif /* GPS_H_ */
