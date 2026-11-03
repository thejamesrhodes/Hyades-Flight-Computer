/* ============================================================================
 * telemetry.c  —  Binary framed telemetry over USB CDC
 *
 * Serialisation is explicit byte-packing (not struct overlay) so the wire
 * layout is fixed regardless of compiler padding. Cortex-M4 is little-endian,
 * so float bytes are copied directly. The transmit is non-blocking: a single
 * CDC_Transmit_FS attempt; on USBD_BUSY the frame is dropped. At 100 Hz a
 * ~110-byte frame (~100 us on USB FS) clears long before the next frame, so a
 * single static buffer is safe (BUSY refuses while a prior frame is in flight).
 * ============================================================================ */
#include "telemetry.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include <string.h>

/* CDC device handle (from usb_device.c). Used only to read TxState so we never
 * overwrite a frame that is still being transmitted. */
extern USBD_HandleTypeDef hUsbDeviceFS;

static bool cdc_tx_busy(void)
{
    USBD_CDC_HandleTypeDef *h = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    /* Not enumerated yet, or a transfer is in flight. */
    return (h == NULL) || (h->TxState != 0u);
}

#define STATE_PAYLOAD_LEN 104u
#define FRAME_MAX_LEN     (4u + STATE_PAYLOAD_LEN + 2u)

static uint8_t  tx_buf[FRAME_MAX_LEN];
static uint32_t dropped = 0u;

/* CRC16-CCITT (XModem): poly 0x1021, init 0xFFFF, no reflection. */
static uint16_t crc16_ccitt(const uint8_t *p, uint32_t n)
{
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0u; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* Little-endian append helpers. */
static uint32_t put_u32(uint8_t *d, uint32_t o, uint32_t v) { memcpy(&d[o], &v, 4); return o+4u; }
static uint32_t put_u16(uint8_t *d, uint32_t o, uint16_t v) { memcpy(&d[o], &v, 2); return o+2u; }
static uint32_t put_f  (uint8_t *d, uint32_t o, float v)    { memcpy(&d[o], &v, 4); return o+4u; }
static uint32_t put_fv (uint8_t *d, uint32_t o, const float *v, int n) {
    for (int i=0;i<n;i++) { o = put_f(d, o, v[i]); } return o;
}

bool Telem_SendState(const TelemState *st)
{
    /* Check BEFORE touching tx_buf: if a prior frame is still in flight, drop
     * this one without disturbing the buffer it is still reading from. */
    if (cdc_tx_busy()) {
        dropped++;
        return false;
    }

    tx_buf[0] = TELEM_SYNC0;
    tx_buf[1] = TELEM_SYNC1;
    tx_buf[2] = TELEM_MSG_STATE;
    tx_buf[3] = (uint8_t)STATE_PAYLOAD_LEN;

    uint32_t o = 4u;
    o = put_u32(tx_buf, o, st->t_us);
    o = put_u32(tx_buf, o, st->frame);
    o = put_fv (tx_buf, o, st->q,     4);
    o = put_fv (tx_buf, o, st->euler, 3);
    o = put_fv (tx_buf, o, st->omega, 3);
    o = put_fv (tx_buf, o, st->accel, 3);
    o = put_fv (tx_buf, o, st->mag,   3);
    o = put_fv (tx_buf, o, st->bias,  3);
    o = put_fv (tx_buf, o, st->fin,   4);
    o = put_u16(tx_buf, o, st->health);
    o = put_u16(tx_buf, o, st->loop_us);

    /* CRC over [MSG_ID, LEN, payload]. */
    uint16_t crc = crc16_ccitt(&tx_buf[2], o - 2u);
    o = put_u16(tx_buf, o, crc);

    if (CDC_Transmit_FS(tx_buf, (uint16_t)o) != USBD_OK) {
        dropped++;
        return false;
    }
    return true;
}

uint32_t Telem_Dropped(void) { return dropped; }

static uint8_t txt_buf[256];

bool Telem_SendText(const char *txt, uint32_t len)
{
    if (cdc_tx_busy()) { dropped++; return false; }
    if (len > sizeof(txt_buf)) { len = sizeof(txt_buf); }
    memcpy(txt_buf, txt, len);
    if (CDC_Transmit_FS(txt_buf, (uint16_t)len) != USBD_OK) { dropped++; return false; }
    return true;
}
