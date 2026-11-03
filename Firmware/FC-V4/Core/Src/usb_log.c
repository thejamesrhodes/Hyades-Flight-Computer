/* ============================================================================
 * usb_log.c  —  USB CDC text logging helpers (see usb_log.h)
 * ============================================================================ */
#include "usb_log.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

static char buf[512];

void USB_Log(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (len <= 0) { return; }
    if (len >= (int)sizeof(buf)) { len = (int)sizeof(buf) - 1; }

    /* Bounded best-effort: 3 tries, 2 ms apart. Boot only. */
    for (uint32_t retry = 0u; retry < 3u; retry++) {
        if (CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len) == USBD_OK) { break; }
        HAL_Delay(2);
    }
}

const char *Log_Fx(char out[LOG_FX_LEN], float v, uint32_t dp)
{
    static const uint32_t p10[5] = { 1u, 10u, 100u, 1000u, 10000u };
    if (dp > 4u) { dp = 4u; }
    if (!isfinite(v)) { (void)snprintf(out, LOG_FX_LEN, "nan"); return out; }

    const float scaled = v * (float)p10[dp];
    if (fabsf(scaled) > 2.0e9f) {
        (void)snprintf(out, LOG_FX_LEN, "%covf", (v < 0.0f) ? '-' : '+');
        return out;
    }
    const int32_t  m = (int32_t)lroundf(scaled);
    const uint32_t a = (uint32_t)((m < 0) ? -m : m);
    const char     s = (m < 0) ? '-' : '+';
    if (dp == 0u) {
        (void)snprintf(out, LOG_FX_LEN, "%c%lu", s, (unsigned long)a);
    } else {
        (void)snprintf(out, LOG_FX_LEN, "%c%lu.%0*lu", s, (unsigned long)(a / p10[dp]),
                       (int)dp, (unsigned long)(a % p10[dp]));
    }
    return out;
}
