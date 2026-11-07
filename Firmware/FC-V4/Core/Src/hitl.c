/* ============================================================================
 * hitl.c  —  see hitl.h. Main-loop context only (no ISR state).
 * ============================================================================ */
#include "board.h"
#if CFG_HITL
#include "hitl.h"
#include "console.h"
#include "crc.h"
#include "gps.h"
#include "telemetry.h"
#include "usbio.h"
#include <string.h>

#define PAYLOAD_MAX   96u
#define SAMPLE_LEN    32u
#define REPLY_LEN     60u
#define POLL_CHUNKS   8u            /* <= 8 x 64 B per call (bounded)            */

typedef enum { S_IDLE, S_SYNC1, S_ID, S_LEN, S_PAY, S_CRC0, S_CRC1 } RxState;

static RxState    s_st;
static uint8_t    s_id, s_len, s_n;
static uint8_t    s_pay[PAYLOAD_MAX];
static uint16_t   s_crc;
static HitlSample s_sample;
static bool       s_pending, s_live;
static uint32_t   s_crc_err, s_overwrite;

static float get_f32(const uint8_t *p) { float v; memcpy(&v, p, 4u); return v; }
static uint32_t get_u32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4u); return v; }

static void dispatch(void)
{
    if ((s_id == HITL_MSG_SAMPLE) && (s_len == SAMPLE_LEN)) {
        if (s_pending) { s_overwrite++; }
        s_sample.idx = get_u32(&s_pay[0]);
        for (uint32_t k = 0u; k < 3u; k++) {
            s_sample.accel[k] = get_f32(&s_pay[4u + (4u * k)]);
            s_sample.gyro[k]  = get_f32(&s_pay[16u + (4u * k)]);
        }
        s_sample.flags = get_u32(&s_pay[28]);
        s_pending = true;
        if (!s_live) { s_live = true; GPS_SetExternal(true); }
    } else if ((s_id == HITL_MSG_NMEA) && s_live) {
        GPS_Inject(s_pay, s_len);
    } else {
        /* unknown id / wrong length: ignored */
    }
}

/* One byte through the frame state machine; returns true if it was consumed
 * by a frame (false = console text). */
static bool rx_byte(uint8_t b)
{
    switch (s_st) {
    case S_IDLE:
        if (b == TELEM_SYNC0) { s_st = S_SYNC1; return true; }
        return false;
    case S_SYNC1:
        s_st = (b == TELEM_SYNC1) ? S_ID : S_IDLE;
        return true;
    case S_ID:
        s_id = b; s_st = S_LEN;
        return true;
    case S_LEN:
        s_len = b; s_n = 0u;
        s_st = (b == 0u) ? S_CRC0 : ((b <= PAYLOAD_MAX) ? S_PAY : S_IDLE);
        return true;
    case S_PAY:
        s_pay[s_n++] = b;
        if (s_n >= s_len) { s_st = S_CRC0; }
        return true;
    case S_CRC0:
        s_crc = b; s_st = S_CRC1;
        return true;
    case S_CRC1: {
        s_crc |= (uint16_t)((uint16_t)b << 8);
        uint8_t hdr[2u + PAYLOAD_MAX];
        hdr[0] = s_id; hdr[1] = s_len;
        memcpy(&hdr[2], s_pay, s_len);
        if (crc16_ccitt(hdr, 2u + (uint32_t)s_len) == s_crc) { dispatch(); } else { s_crc_err++; }
        s_st = S_IDLE;
        return true;
    }
    default:
        s_st = S_IDLE;
        return true;
    }
}

void Hitl_Poll(void)
{
    for (uint32_t c = 0u; c < POLL_CHUNKS; c++) {
        uint8_t buf[64], txt[64];
        const uint32_t n = UsbIo_Read(buf, sizeof buf);
        if (n == 0u) { break; }
        uint32_t nt = 0u;
        for (uint32_t i = 0u; i < n; i++) {
            if (!rx_byte(buf[i])) { txt[nt++] = buf[i]; }
        }
        if (nt > 0u) { Con_Feed(txt, nt); }
        if (s_pending) { break; }               /* lockstep: run the frame first */
    }
}

bool Hitl_Live(void)    { return s_live; }
bool Hitl_Pending(void) { return s_pending; }

bool Hitl_Take(HitlSample *s)
{
    if (!s_pending) { return false; }
    *s = s_sample;
    s_pending = false;
    return true;
}

static uint32_t put(uint8_t *d, uint32_t o, const void *v, uint32_t n) { memcpy(&d[o], v, n); return o + n; }

bool Hitl_Reply(const HitlReply *r)
{
    uint8_t f[4u + REPLY_LEN + 2u];
    f[0] = TELEM_SYNC0; f[1] = TELEM_SYNC1; f[2] = HITL_MSG_REPLY; f[3] = (uint8_t)REPLY_LEN;
    uint32_t o = 4u;
    o = put(f, o, &r->idx, 4u);
    o = put(f, o, &r->phase, 1u);
    o = put(f, o, &r->mode, 1u);
    o = put(f, o, &r->faults, 2u);
    o = put(f, o, &r->loop_us, 2u);
    o = put(f, o, &r->nav_flags, 1u);
    o = put(f, o, &r->hitl_flags, 1u);
    o = put(f, o, r->fin, 16u);
    o = put(f, o, r->q, 16u);
    o = put(f, o, &r->h, 4u);
    o = put(f, o, &r->v_up, 4u);
    o = put(f, o, &r->speed, 4u);
    o = put(f, o, &r->qbar, 4u);
    const uint16_t crc = crc16_ccitt(&f[2], o - 2u);
    o = put(f, o, &crc, 2u);
    return UsbIo_Write(f, o);
}

uint32_t Hitl_CrcErrors(void) { return s_crc_err; }
uint32_t Hitl_Overwrites(void) { return s_overwrite; }

#endif /* CFG_HITL */
