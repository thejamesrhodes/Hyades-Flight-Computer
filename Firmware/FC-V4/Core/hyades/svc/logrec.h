/* ============================================================================
 * logrec.h  —  Fixed 48-byte flight-log records (format version 1)
 *
 * Every record: byte 0 = type, bytes 46..47 = CRC16-CCITT (crc.h) over bytes
 * 0..45, little-endian. Erased flash (0xFF) is never a valid type, so the end
 * of the log is the first slot whose type byte is 0xFF. A torn record (power
 * lost mid-write) fails its CRC and is skipped by the reader.
 *
 * STATE  (type 0x51)                    EVENT (type 0xE5)       BOOT (type 0xB7)
 *   0 u8  type      1 u8 phase idx        1 u8  code              1 u8  version
 *   2 u8  ctrl mode 3 u8 flags (LOGF_*)   2 u8  phase idx         2 u16 frame Hz
 *   4 u32 t_us      8 u16 seq             4 u32 t_us  8 u16 seq   4 u32 t_us  8 u16 seq
 *  10 i16 q[4]      x 1/32767             12 u32 arg0             10 u16 log Hz
 *  18 i16 w[3]      dps x 10              16 u32 arg1             12 char build[32]
 *  24 i16 a[3]      m/s^2 x 100           20 char text[26]
 *  30 i16 bias[3]   dps x 100
 *  36 i8  fin[4]    x 127
 *  40 u16 loop_us   42 u16 fault bits (active | latch)   44 u16 reserved
 *
 * Out-of-range or non-finite values saturate; NaN is stored as INT16_MIN.
 * Decoder: Tools/fc_pull.py. Pure module.
 * ============================================================================ */
#ifndef LOGREC_H_
#define LOGREC_H_

#include <stdint.h>
#include <stdbool.h>

#define LOGREC_SIZE      48u
#define LOGREC_VERSION   1u
#define LOGREC_STATE     0x51u
#define LOGREC_EVENT     0xE5u
#define LOGREC_BOOT      0xB7u

#define LOGF_GYRO_OK     (1u << 0)
#define LOGF_ACCEL_OK    (1u << 1)
#define LOGF_ACC_APPLIED (1u << 2)
#define LOGF_OVERRUN     (1u << 3)
#define LOGF_GPS_FIX     (1u << 4)

typedef struct {
    uint32_t t_us;
    uint16_t seq;
    uint8_t  phase, mode, flags;
    float    q[4], w_dps[3], a_ms2[3], bias_dps[3], fin[4];
    uint16_t loop_us, faults;
} LogState;

typedef struct {
    uint32_t t_us;
    uint16_t seq;
    uint8_t  code, phase;
    uint32_t arg0, arg1;
    char     text[26];          /* NUL-padded, need not be terminated */
} LogEvent;

typedef struct {
    uint32_t t_us;
    uint16_t seq;
    uint16_t frame_hz, log_hz;
    char     build[32];         /* NUL-padded */
} LogBoot;

void LogRec_PackState(uint8_t out[LOGREC_SIZE], const LogState *s);
void LogRec_PackEvent(uint8_t out[LOGREC_SIZE], const LogEvent *e);
void LogRec_PackBoot (uint8_t out[LOGREC_SIZE], const LogBoot *b);

/* true if the type is known and the CRC matches */
bool LogRec_Valid(const uint8_t rec[LOGREC_SIZE]);

/* Inverse of PackState (quantised). For host tests and on-target readback. */
bool LogRec_UnpackState(const uint8_t rec[LOGREC_SIZE], LogState *s);

#endif /* LOGREC_H_ */
