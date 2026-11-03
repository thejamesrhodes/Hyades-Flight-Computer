/* ============================================================================
 * usbio.h  —  USB CDC byte transport (the only module that calls CDC_*)
 *
 * TX: every producer (console replies, status text, binary telemetry, log
 *     pull) writes into one 4 KB ring; UsbIo_Task() drains it from the main
 *     loop. Writes never block: a write that does not fit is dropped whole and
 *     counted. Nothing is sent until a host has opened the port (it sets the
 *     line coding, asserts DTR, or sends a byte), so the boot log waits in the
 *     ring and appears when Tera Term / a script / the CubeIDE terminal
 *     connects. DTR dropping marks the port closed again.
 * RX: the CDC receive ISR copies packets into a 512-byte ring (or into the
 *     firmware-update staging buffer while an update is in progress); the
 *     console reads it from the main loop.
 * ============================================================================ */
#ifndef USBIO_H_
#define USBIO_H_

#include <stdint.h>
#include <stdbool.h>

void     UsbIo_Init(void);

/* Queue bytes for transmission. Returns false (nothing queued) if they do not fit. */
bool     UsbIo_Write(const void *data, uint32_t len);

/* Exclusive mode (log pull): ordinary writes are dropped so nothing can be
 * interleaved with a raw binary stream; the owner uses UsbIo_WriteExcl(). */
void     UsbIo_SetExclusive(bool on);
bool     UsbIo_WriteExcl(const void *data, uint32_t len);

/* Free space in the TX ring (bytes). */
uint32_t UsbIo_TxFree(void);

/* True when the TX ring is empty and the endpoint idle (for raw streaming). */
bool     UsbIo_TxIdle(void);

/* Hand a buffer straight to the endpoint (no copy). Only when UsbIo_TxIdle().
 * The buffer must stay valid until the next UsbIo_TxIdle(). Max 4096 bytes. */
bool     UsbIo_SendRaw(const uint8_t *data, uint32_t len);

/* Drain the ring. Call every frame. Bounded: at most one USB transfer. */
void     UsbIo_Task(void);

/* Read up to max bytes received from the host. Returns the count. */
uint32_t UsbIo_Read(uint8_t *dst, uint32_t max);

/* Host has the port open (enumerated, and opened per above). */
bool     UsbIo_Connected(void);

/* Diagnostics */
uint32_t UsbIo_TxDropped(void);
uint32_t UsbIo_RxDropped(void);

/* Called from usbd_cdc_if.c (USB interrupt context). */
void     UsbIo_RxIsr(const uint8_t *buf, uint32_t len);
void     UsbIo_SetDtr(bool dtr);
void     UsbIo_HostOpened(void);

#endif /* USBIO_H_ */
