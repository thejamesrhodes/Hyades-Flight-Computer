/* ============================================================================
 * nmea.h  —  NMEA-0183 sentence parser (GGA + RMC), byte-at-a-time
 *
 * Pure: the caller supplies bytes and the current time (ms); no UART, no HAL.
 * Bounded: one byte per call, fixed 96-byte line buffer, no heap (numbers are
 * parsed by hand: newlib strtod/strtof allocate).
 * Accepts any talker ID ($GPGGA, $GNGGA, $BDGGA ...). Sentences failing the
 * XOR checksum, or longer than the buffer, are counted and dropped.
 * ============================================================================ */
#ifndef NMEA_H_
#define NMEA_H_

#include <stdint.h>
#include <stdbool.h>

#define NMEA_LINE_MAX 96u           /* NMEA max is 82 chars + margin           */

typedef struct {
    /* GGA */
    uint8_t  fix_quality;     /* 0 none, 1 GPS, 2 DGPS, ... (GGA field 6)  */
    uint8_t  sats;            /* satellites used                           */
    float    hdop;
    float    alt_msl_m;
    double   lat_deg, lon_deg;/* +N / +E (double: float has ~1 m at 50 deg) */
    uint32_t utc_hhmmss;
    /* RMC */
    bool     rmc_valid;       /* 'A' = valid                               */
    float    speed_mps;
    float    course_deg;
    /* bookkeeping (caller's ms clock) */
    uint32_t last_gga_ms;
    uint32_t last_rx_ms;      /* last checksum-valid sentence              */
    /* diagnostics */
    uint32_t bytes;
    uint32_t sentences;       /* checksum-valid                            */
    uint32_t gga, rmc;
    uint32_t cksum_err;       /* bad checksum or over-long line            */
} GpsData;

typedef struct {
    char     line[NMEA_LINE_MAX];
    uint32_t len;
    bool     in_sentence;
} NmeaParser;

void Nmea_Init(NmeaParser *p, GpsData *g);
void Nmea_Feed(NmeaParser *p, GpsData *g, char c, uint32_t now_ms);

bool Nmea_Alive (const GpsData *g, uint32_t now_ms, uint32_t timeout_ms);
bool Nmea_HasFix(const GpsData *g, uint32_t now_ms, uint32_t timeout_ms);

/* Exposed for unit tests: decimal text -> double, no heap. */
double Nmea_ParseNum(const char *v);

#endif /* NMEA_H_ */
