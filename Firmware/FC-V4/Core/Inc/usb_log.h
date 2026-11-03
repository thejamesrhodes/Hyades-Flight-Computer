/* ============================================================================
 * usb_log.h  —  USB CDC text logging helpers
 *
 * USB_Log: printf-style, may briefly retry -> BOOT / DIAGNOSTIC USE ONLY,
 * never from the 100 Hz frame (the frame uses Telem_SendText, non-blocking).
 * Float printf support is NOT linked (it can allocate and is large); format
 * floats with Log_Fx(), which returns fixed-point text such as "+12.3".
 * ============================================================================ */
#ifndef USB_LOG_H_
#define USB_LOG_H_

#include <stdint.h>

#define LOG_FX_LEN  24u   /* buffer size for one Log_Fx() result */

void USB_Log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Write v with dp (0..4) decimals and an explicit sign into buf; returns buf.
 * Non-finite or out-of-range values print as "nan" / "+ovf" / "-ovf". */
const char *Log_Fx(char buf[LOG_FX_LEN], float v, uint32_t dp);

#endif /* USB_LOG_H_ */
