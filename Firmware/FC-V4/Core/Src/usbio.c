/* ============================================================================
 * usbio.c  —  USB CDC byte transport (see usbio.h)
 * ============================================================================ */
#include "usbio.h"
#include "fwupdate.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

#define TX_RING   4096u          /* power of two */
#define TX_CHUNK  512u           /* bytes per USB transfer from the ring */
#define RX_RING   512u           /* power of two */

/* TX ring: main-loop only (producer and consumer), so no locking. */
static uint8_t  s_tx[TX_RING];
static uint32_t s_tx_head, s_tx_tail;
static uint8_t  s_tx_lin[TX_CHUNK];   /* stable copy handed to the endpoint */
static uint32_t s_tx_dropped;

/* RX ring: written by the USB ISR (head), read by the main loop (tail). Single
 * producer / single consumer, 32-bit indices: atomic on Cortex-M4. */
static uint8_t           s_rx[RX_RING];
static volatile uint32_t s_rx_head, s_rx_tail;
static volatile uint32_t s_rx_dropped;

static volatile bool s_open;     /* host has the port open (see usbio.h) */
static volatile bool s_dtr;
static bool s_excl;

static bool ep_busy(void)
{
    const USBD_CDC_HandleTypeDef *h = (const USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    return (h == NULL) || (h->TxState != 0u);
}

void UsbIo_Init(void)
{
    s_tx_head = s_tx_tail = 0u;
    s_rx_head = s_rx_tail = 0u;
    s_tx_dropped = 0u; s_rx_dropped = 0u;
    s_dtr = false;
    s_open = false;
    s_excl = false;
}

bool UsbIo_Connected(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) && s_open;
}

uint32_t UsbIo_TxFree(void) { return (TX_RING - 1u) - ((s_tx_head - s_tx_tail) & (TX_RING - 1u)); }

void UsbIo_SetExclusive(bool on) { s_excl = on; }

bool UsbIo_Write(const void *data, uint32_t len)
{
    if (s_excl) { s_tx_dropped += len; return false; }
    return UsbIo_WriteExcl(data, len);
}

bool UsbIo_WriteExcl(const void *data, uint32_t len)
{
    if (len > UsbIo_TxFree()) { s_tx_dropped += len; return false; }
    const uint8_t *p = (const uint8_t *)data;
    for (uint32_t i = 0u; i < len; i++) {
        s_tx[s_tx_head] = p[i];
        s_tx_head = (s_tx_head + 1u) & (TX_RING - 1u);
    }
    return true;
}

bool UsbIo_TxIdle(void) { return (s_tx_head == s_tx_tail) && !ep_busy(); }

bool UsbIo_SendRaw(const uint8_t *data, uint32_t len)
{
    if (!UsbIo_Connected() || !UsbIo_TxIdle() || (len == 0u) || (len > 4096u)) { return false; }
    /* CDC_Transmit_FS takes a non-const pointer but only reads the buffer. */
    return CDC_Transmit_FS((uint8_t *)(uintptr_t)data, (uint16_t)len) == USBD_OK;
}

void UsbIo_Task(void)
{
    if (!UsbIo_Connected() || ep_busy() || (s_tx_head == s_tx_tail)) { return; }
    uint32_t n = 0u;
    while ((s_tx_tail != s_tx_head) && (n < TX_CHUNK)) {
        s_tx_lin[n++] = s_tx[s_tx_tail];
        s_tx_tail = (s_tx_tail + 1u) & (TX_RING - 1u);
    }
    if (CDC_Transmit_FS(s_tx_lin, (uint16_t)n) != USBD_OK) { s_tx_dropped += n; }
}

uint32_t UsbIo_Read(uint8_t *dst, uint32_t max)
{
    uint32_t n = 0u;
    while ((n < max) && (s_rx_tail != s_rx_head)) {
        dst[n++] = s_rx[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1u) & (RX_RING - 1u);
    }
    return n;
}

void UsbIo_RxIsr(const uint8_t *buf, uint32_t len)
{
    s_open = true;                                 /* the host is talking to us */
    if (Fw_RxIsr(buf, len)) { return; }            /* image bytes during an update */
    for (uint32_t i = 0u; i < len; i++) {
        const uint32_t next = (s_rx_head + 1u) & (RX_RING - 1u);
        if (next == s_rx_tail) { s_rx_dropped++; continue; }
        s_rx[s_rx_head] = buf[i];
        s_rx_head = next;
    }
}

void UsbIo_SetDtr(bool dtr)
{
    if (dtr)        { s_open = true; }
    else if (s_dtr) { s_open = false; }       /* falling edge: host closed */
    s_dtr = dtr;
}

void UsbIo_HostOpened(void) { s_open = true; }

uint32_t UsbIo_TxDropped(void) { return s_tx_dropped; }
uint32_t UsbIo_RxDropped(void) { return s_rx_dropped; }
