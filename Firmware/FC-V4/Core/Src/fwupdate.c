/* ============================================================================
 * fwupdate.c  —  see fwupdate.h
 * ============================================================================ */
#include "fwupdate.h"
#include "board.h"
#include "console.h"
#include "crc.h"
#include "wdg.h"
#include "main.h"
#include "usbd_core.h"
#include <stdlib.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

#define FW_MIN_SIZE   1024u
#define RAM_BASE      0x20000000u
#define RAM_END       0x20020000u        /* 128 KB */
#define PROBE_MAGIC   0x00C0FFEEu

typedef enum { FW_IDLE = 0, FW_RECEIVING, FW_VERIFY, FW_ARMED } FwState;

/* Staging buffer (.bss): word-aligned because it is programmed as words. */
static uint8_t s_stage[FW_STAGE_SIZE] __attribute__((aligned(4)));
typedef char fw_stage_size_check[((FW_STAGE_SIZE & 3u) == 0u) ? 1 : -1];

static volatile FwState  s_state = FW_IDLE;
static volatile uint32_t s_expected, s_received, s_last_rx_ms;
static uint32_t          s_want_crc, s_fire_at_ms;

/* ---------------------------------------------------------------------------
 * RAM-resident flasher. Load-bearing constraints (same as GMWM, RM0383 §3):
 *  - executes from SRAM: flash cannot be read while it is being erased;
 *  - long_call: SRAM is >16 MB from flash, out of BL range;
 *  - calls nothing (all HAL/CMSIS non-inline code is in flash) and reads no
 *    const data (it may be erased); literal pools land in .RamFunc;
 *  - interrupts disabled by the caller (a vector fetch would read flash);
 *  - kicks the (stretched) IWDG between operations.
 * Erases sectors 0..k covering nbytes (sector sizes 16,16,16,16,64 KB), never
 * the log sectors. Returns FLASH_SR error bits on failure; resets on success.
 * ------------------------------------------------------------------------- */
#define SR_ERRORS (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR)

__attribute__((section(".RamFunc"), noinline, long_call))
static uint32_t fw_flash_and_reset(const uint32_t *src, uint32_t nwords, uint32_t probe)
{
    if (probe != 0u) { return PROBE_MAGIC; }

    volatile uint32_t *dst = (volatile uint32_t *)APP_FLASH_BASE;
    const uint32_t nbytes = nwords * 4u;
    uint32_t sr;

    if ((FLASH->CR & FLASH_CR_LOCK) != 0u) { FLASH->KEYR = 0x45670123u; FLASH->KEYR = 0xCDEF89ABu; }
    while ((FLASH->SR & FLASH_SR_BSY) != 0u) { IWDG->KR = 0xAAAAu; }
    FLASH->SR = FLASH_SR_EOP | SR_ERRORS;

    for (uint32_t s = 0u; s < 5u; s++) {
        FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER | (s << FLASH_CR_SNB_Pos);
        FLASH->CR |= FLASH_CR_STRT;
        __DSB();
        while ((FLASH->SR & FLASH_SR_BSY) != 0u) { IWDG->KR = 0xAAAAu; }
        sr = FLASH->SR & SR_ERRORS;
        if (sr != 0u) { FLASH->CR = FLASH_CR_LOCK; return sr; }
        /* End offset of sector s, computed (a const table would live in flash). */
        const uint32_t end = (s < 4u) ? ((s + 1u) * 0x4000u) : 0x20000u;
        if (end >= nbytes) { break; }
    }

    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
    __DSB();
    for (uint32_t i = 0u; i < nwords; i++) {
        dst[i] = src[i];
        __DSB();
        while ((FLASH->SR & FLASH_SR_BSY) != 0u) { }
        sr = FLASH->SR & SR_ERRORS;
        if (sr != 0u) { FLASH->CR = FLASH_CR_LOCK; return sr; }
        if ((i & 0x3FFu) == 0u) { IWDG->KR = 0xAAAAu; }
    }
    FLASH->CR = FLASH_CR_LOCK;

    /* Flush the ART data cache so the verify reads the array, not stale lines. */
    FLASH->ACR &= ~FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    for (uint32_t i = 0u; i < nwords; i++) {
        if (dst[i] != src[i]) { return 0xBADC0DE0u; }
    }

    __DSB();
    SCB->AIRCR = (0x5FAu << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
    __DSB();
    for (;;) { }
}

static bool vectors_plausible(const uint8_t *img, uint32_t len, const char **why)
{
    uint32_t msp, pc;
    if (len < 8u) { *why = "too-short"; return false; }
    memcpy(&msp, &img[0], 4u);
    memcpy(&pc,  &img[4], 4u);
    if ((msp < RAM_BASE) || (msp > RAM_END)) { *why = "sp-not-in-sram"; return false; }
    if ((pc < APP_FLASH_BASE) || (pc >= (APP_FLASH_BASE + APP_FLASH_SIZE))) { *why = "reset-not-in-app-flash"; return false; }
    if ((pc & 1u) == 0u) { *why = "reset-not-thumb"; return false; }
    return true;
}

static void abort_with(const char *why)
{
    s_state = FW_IDLE;
    Con_Printf("FWABORT %s\r\n", why);
}

void Fw_Command(uint32_t argc, char *argv[])
{
    if (s_state != FW_IDLE) { Con_Printf("FWABORT busy\r\n"); return; }
    if (argc < 3u) {
        Con_Printf("usage: fw <size> <crc32hex>   (max %lu bytes; use Tools/fc_flash.py)\r\n",
                   (unsigned long)FW_STAGE_SIZE);
        return;
    }
    char *end = NULL;
    const uint32_t size = (uint32_t)strtoul(argv[1], &end, 0);
    if ((end == argv[1]) || (*end != '\0')) { Con_Printf("FWABORT bad-size\r\n"); return; }
    const uint32_t crc = (uint32_t)strtoul(argv[2], &end, 16);
    if ((end == argv[2]) || (*end != '\0')) { Con_Printf("FWABORT bad-crc\r\n"); return; }
    if ((size < FW_MIN_SIZE) || (size > FW_STAGE_SIZE)) {
        Con_Printf("FWABORT size-out-of-range %lu (max %lu)\r\n", (unsigned long)size, (unsigned long)FW_STAGE_SIZE);
        return;
    }
    s_expected = size; s_want_crc = crc; s_received = 0u;
    s_last_rx_ms = HAL_GetTick();
    s_state = FW_RECEIVING;                                  /* arms Fw_RxIsr */
    Con_Printf("FWREADY %lu\r\n", (unsigned long)size);
}

bool Fw_RxIsr(const uint8_t *buf, uint32_t len)
{
    if (s_state != FW_RECEIVING) { return false; }
    const uint32_t room = s_expected - s_received;
    const uint32_t n = (len < room) ? len : room;            /* overshoot discarded -> CRC fails */
    memcpy(&s_stage[s_received], buf, n);
    s_received += n;
    s_last_rx_ms = HAL_GetTick();
    if (s_received >= s_expected) { s_state = FW_VERIFY; }
    return true;
}

bool Fw_Busy(void) { return s_state != FW_IDLE; }

void Fw_Task(void)
{
    switch (s_state) {
    case FW_IDLE:
        break;

    case FW_RECEIVING:
        if ((HAL_GetTick() - s_last_rx_ms) > FW_RX_TIMEOUT_MS) {
            Con_Printf("FWABORT timeout after %lu of %lu bytes\r\n",
                       (unsigned long)s_received, (unsigned long)s_expected);
            s_state = FW_IDLE;
        }
        break;

    case FW_VERIFY: {
        const char *why = "";
        Con_Printf("FWRECV %lu\r\n", (unsigned long)s_received);
        const uint32_t got = crc32_ieee(s_stage, s_expected);
        if (got != s_want_crc) {
            Con_Printf("FWCRC bad got=%08lX want=%08lX\r\n", (unsigned long)got, (unsigned long)s_want_crc);
            s_state = FW_IDLE;
            return;
        }
        Con_Printf("FWCRC ok\r\n");
        if (!vectors_plausible(s_stage, s_expected, &why)) { Con_Printf("FWVEC bad %s\r\n", why); s_state = FW_IDLE; return; }
        Con_Printf("FWVEC ok\r\n");
        /* Prove instructions can be fetched from .RamFunc before erasing anything. */
        if (fw_flash_and_reset(NULL, 0u, 1u) != PROBE_MAGIC) { abort_with("ramfunc-probe"); return; }
        while ((s_expected & 3u) != 0u) { s_stage[s_expected++] = 0xFFu; }   /* pad final word */
        Con_Printf("FWPROG\r\n");
        s_fire_at_ms = HAL_GetTick() + 200u;                 /* let the replies drain */
        s_state = FW_ARMED;
        break;
    }

    case FW_ARMED:
        if ((int32_t)(HAL_GetTick() - s_fire_at_ms) < 0) { break; }
        (void)USBD_Stop(&hUsbDeviceFS);                      /* clean detach for the host */
        Wdg_Stretch();
        HAL_Delay(20);
        __disable_irq();
        {
            const uint32_t err = fw_flash_and_reset((const uint32_t *)(const void *)s_stage,
                                                    s_expected / 4u, 0u);
            /* Only reached on failure: flash is blank or partial. Nothing to
             * return to; spin until the watchdog resets into whatever is left.
             * Recovery is SWD (STM32CubeProgrammer). */
            (void)err;
            for (;;) { }
        }

    default:
        s_state = FW_IDLE;
        break;
    }
}
