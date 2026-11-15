/* ============================================================================
 * fincal.h  —  Canard zero calibration record and journal (CAL-01)
 *
 * The calibration is the angle, at the servo, that puts each canard on its
 * aerodynamic zero. It is kept in an append-only journal of fixed 32-byte
 * records at the start of log sector 5 (board.h CAL_FLASH_*):
 *
 *   0  magic   u32  'FCAL'
 *   4  seq     u16  increments on every save
 *   6  version u8   FINCAL_VERSION (versions 2 and 3 are read and converted)
 *   7  count   u8   4 (fins)
 *   8  off     i32[4]  millidegrees, fin 0 top, 1 right, 2 bottom, 3 left
 *  24  spare   u8[4]   zero
 *  28  crc32   u32  IEEE over bytes 0..27
 *
 * A save appends one record: no erase, so it never stalls the frame for more
 * than one record's programming (~0.13 ms). The newest valid record wins; a
 * torn or foreign record is skipped, never trusted. Pure module: the scan
 * takes a pointer, so it runs on a host buffer in the unit tests.
 * ============================================================================ */
#ifndef FINCAL_H_
#define FINCAL_H_

#include <stdint.h>
#include <stdbool.h>

#define FINCAL_REC_SIZE    32u
#define FINCAL_MAGIC       0x4C414346u     /* "FCAL" little-endian */
#define FINCAL_VERSION     4u      /* 4: degrees at 10.34 us/deg (SERVO_RANGE_DEG 116, from 15 Nov 2026), +/-56 deg */
#define FINCAL_RANGE_DEG   116     /* the scale of FINCAL_VERSION; pwm.c checks it against board.h */
/* Older records are still read. Each zero is converted to the same pulse (x FINCAL_RANGE_DEG / old
 * range), so an alignment made at an older scale holds:
 *   3: degrees at 8.889 us/deg (range 135, 10-15 Nov 2026), +/-65 deg;
 *   2: degrees at 6.667 us/deg (range 180, before 10 Nov 2026), +/-90 deg. */
#define FINCAL_V3          3u
#define FINCAL_V3_RANGE_DEG   135
#define FINCAL_V3_LIMIT_MDEG  65000
#define FINCAL_V2          2u
#define FINCAL_V2_RANGE_DEG   180
#define FINCAL_V2_LIMIT_MDEG  90000
#define FINCAL_FINS        4u
#define FINCAL_LIMIT_MDEG  56000           /* +/-56 deg: the neutral pulse stays inside SERVO_MIN..MAX_US (pwm.c) */

typedef struct {
    bool     valid;
    uint16_t seq;
    uint8_t  version;                  /* the record it was read from (not FINCAL_VERSION: converted) */
    int32_t  off_mdeg[FINCAL_FINS];    /* always at the current scale */
} FinCal;

typedef struct {
    FinCal   cal;      /* newest valid record (cal.valid false if none)          */
    uint32_t next;     /* first slot of the erased tail: where the next save goes */
    uint32_t slots;    /* capacity in records                                     */
    uint32_t bad;      /* non-erased slots that are not valid records             */
} FinCalScan;

void FinCal_Pack(uint8_t out[FINCAL_REC_SIZE], uint16_t seq, const int32_t off_mdeg[FINCAL_FINS]);
/* true if magic, version, count, CRC and the +/-limit all check */
bool FinCal_Unpack(const uint8_t in[FINCAL_REC_SIZE], FinCal *out);
bool FinCal_InRange(const int32_t off_mdeg[FINCAL_FINS]);
/* every |zero| <= lim_mdeg (e.g. the limit that keeps full command authority) */
bool FinCal_Within(const int32_t off_mdeg[FINCAL_FINS], int32_t lim_mdeg);
void FinCal_Scan(const uint8_t *base, uint32_t size, FinCalScan *s);

#endif /* FINCAL_H_ */
