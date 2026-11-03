/* ============================================================================
 * telemetry.c  —  Binary framed STATE telemetry (see telemetry.h)
 *
 * Serialisation is explicit byte-packing (not struct overlay) so the wire
 * layout is fixed regardless of compiler padding. Cortex-M4 is little-endian,
 * so float bytes are copied directly.
 * ============================================================================ */
#include "telemetry.h"
#include "usbio.h"
#include "crc.h"
#include <string.h>

#define STATE_PAYLOAD_LEN 104u
#define FRAME_MAX_LEN     (4u + STATE_PAYLOAD_LEN + 2u)

static uint32_t put_u32(uint8_t *d, uint32_t o, uint32_t v) { memcpy(&d[o], &v, 4u); return o + 4u; }
static uint32_t put_u16(uint8_t *d, uint32_t o, uint16_t v) { memcpy(&d[o], &v, 2u); return o + 2u; }
static uint32_t put_fv (uint8_t *d, uint32_t o, const float *v, uint32_t n)
{
    memcpy(&d[o], v, n * 4u);
    return o + (n * 4u);
}

bool Telem_SendImu(uint32_t ts, int16_t temp, const int16_t a[3], const int16_t g[3])
{
    uint8_t f[4u + 18u + 2u];
    f[0] = TELEM_SYNC0; f[1] = TELEM_SYNC1; f[2] = TELEM_MSG_IMU; f[3] = 18u;
    uint32_t o = put_u32(f, 4u, ts);
    o = put_u16(f, o, (uint16_t)temp);
    for (uint32_t k = 0u; k < 3u; k++) { o = put_u16(f, o, (uint16_t)a[k]); }
    for (uint32_t k = 0u; k < 3u; k++) { o = put_u16(f, o, (uint16_t)g[k]); }
    o = put_u16(f, o, crc16_ccitt(&f[2], o - 2u));
    return UsbIo_Write(f, o);
}

bool Telem_SendState(const TelemState *st)
{
    uint8_t f[FRAME_MAX_LEN];
    f[0] = TELEM_SYNC0;
    f[1] = TELEM_SYNC1;
    f[2] = TELEM_MSG_STATE;
    f[3] = (uint8_t)STATE_PAYLOAD_LEN;

    uint32_t o = 4u;
    o = put_u32(f, o, st->t_us);
    o = put_u32(f, o, st->frame);
    o = put_fv (f, o, st->q,     4u);
    o = put_fv (f, o, st->euler, 3u);
    o = put_fv (f, o, st->omega, 3u);
    o = put_fv (f, o, st->accel, 3u);
    o = put_fv (f, o, st->mag,   3u);
    o = put_fv (f, o, st->bias,  3u);
    o = put_fv (f, o, st->fin,   4u);
    o = put_u16(f, o, st->health);
    o = put_u16(f, o, st->loop_us);
    o = put_u16(f, o, crc16_ccitt(&f[2], o - 2u));     /* over [MSG_ID, LEN, payload] */
    return UsbIo_Write(f, o);
}
