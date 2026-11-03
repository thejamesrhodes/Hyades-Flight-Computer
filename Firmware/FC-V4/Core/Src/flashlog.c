/* ============================================================================
 * flashlog.c  —  see flashlog.h. Register-level flash access (RM0383 §3).
 * ============================================================================ */
#include "flashlog.h"
#include "board.h"
#include "crc.h"
#include "usbio.h"
#include "wdg.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#define SLOTS       (LOG_FLASH_SIZE / LOGREC_SIZE)
#define SR_ERRORS   (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR)
#define PULL_CHUNK  2048u
#define BSY_SPIN_PROGRAM 20000u        /* >> 16 us word program at 96 MHz        */
#define BSY_SPIN_ERASE   400000000u    /* >> 2 s sector erase (bound, not timing) */

typedef char logsize_check[((LOG_FLASH_SIZE % LOGREC_SIZE) == 0u) ? 1 : -1];

static uint32_t s_next;           /* next free slot index */
static bool     s_active;
static uint16_t s_seq;
static uint32_t s_errors;

typedef enum { PULL_IDLE = 0, PULL_HEADER, PULL_DATA, PULL_TRAILER } PullState;
static PullState s_pull;
static uint32_t  s_pull_off, s_pull_len, s_pull_crc;

static const uint8_t *slot_addr(uint32_t i) { return (const uint8_t *)(LOG_FLASH_BASE + (i * LOGREC_SIZE)); }

static bool wait_bsy(uint32_t spin)
{
    for (uint32_t i = 0u; i < spin; i++) {
        if ((FLASH->SR & FLASH_SR_BSY) == 0u) { return (FLASH->SR & SR_ERRORS) == 0u; }
    }
    return false;
}

static void unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0u) { FLASH->KEYR = 0x45670123u; FLASH->KEYR = 0xCDEF89ABu; }
    FLASH->SR = FLASH_SR_EOP | SR_ERRORS;
}

static void lock(void) { FLASH->CR = FLASH_CR_LOCK; }

/* Invalidate the ART data cache after erase so reads see the array. */
static void flush_dcache(void)
{
    FLASH->ACR &= ~FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |= FLASH_ACR_DCEN;
}

void FlashLog_Init(void)
{
    s_active = false; s_errors = 0u; s_pull = PULL_IDLE;
    s_next = SLOTS;
    for (uint32_t i = 0u; i < SLOTS; i++) {
        if (slot_addr(i)[0] == 0xFFu) { s_next = i; break; }
    }
    /* Continue the sequence after the last valid record (if any). */
    s_seq = 0u;
    if (s_next > 0u) {
        const uint8_t *r = slot_addr(s_next - 1u);
        if (LogRec_Valid(r)) { s_seq = (uint16_t)(r[8] | ((uint16_t)r[9] << 8)); s_seq++; }
    }
}

uint16_t FlashLog_NextSeq(void) { return s_seq; }
bool     FlashLog_Active(void)  { return s_active; }
uint32_t FlashLog_UsedBytes(void)     { return s_next * LOGREC_SIZE; }
uint32_t FlashLog_CapacityBytes(void) { return LOG_FLASH_SIZE; }
uint32_t FlashLog_WriteErrors(void)   { return s_errors; }

static bool program_record(const uint8_t rec[LOGREC_SIZE])
{
    if (s_next >= SLOTS) { return false; }
    volatile uint32_t *dst = (volatile uint32_t *)(LOG_FLASH_BASE + (s_next * LOGREC_SIZE));
    bool ok = true;
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
    for (uint32_t w = 0u; (w < (LOGREC_SIZE / 4u)) && ok; w++) {
        uint32_t v;
        memcpy(&v, &rec[w * 4u], 4u);
        dst[w] = v;
        __DSB();
        ok = wait_bsy(BSY_SPIN_PROGRAM);
    }
    FLASH->CR &= ~FLASH_CR_PG;
    s_next++;                       /* a torn slot is skipped, never rewritten */
    if (!ok) { s_errors++; FLASH->SR = SR_ERRORS; }
    s_seq++;
    return ok;
}

bool FlashLog_Start(const LogBoot *boot)
{
    if (s_active) { return true; }
    if (s_next >= SLOTS) { return false; }
    unlock();
    s_active = true;
    LogBoot b = *boot;
    b.seq = s_seq;
    uint8_t rec[LOGREC_SIZE];
    LogRec_PackBoot(rec, &b);
    return FlashLog_Write(rec);
}

void FlashLog_Stop(void)
{
    if (!s_active) { return; }
    s_active = false;
    lock();
}

bool FlashLog_Write(const uint8_t rec[LOGREC_SIZE])
{
    if (!s_active) { return false; }
    if (s_next >= SLOTS) { FlashLog_Stop(); return false; }
    return program_record(rec);
}

bool FlashLog_Erase(void)
{
    if (s_pull != PULL_IDLE) { return false; }
    FlashLog_Stop();
    Wdg_Stretch();
    unlock();
    bool ok = true;
    for (uint32_t k = 0u; (k < LOG_SECTOR_COUNT) && ok; k++) {
        FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER | ((LOG_FIRST_SECTOR + k) << FLASH_CR_SNB_Pos);
        FLASH->CR |= FLASH_CR_STRT;
        __DSB();
        ok = wait_bsy(BSY_SPIN_ERASE);
        Wdg_Kick();
    }
    lock();
    flush_dcache();
    Wdg_Restore();
    s_next = 0u; s_seq = 0u;
    return ok;
}

bool FlashLog_PullStart(void)
{
    if (s_pull != PULL_IDLE) { return false; }
    FlashLog_Stop();
    s_pull_off = 0u; s_pull_len = FlashLog_UsedBytes(); s_pull_crc = 0u;
    s_pull = PULL_HEADER;
    return true;
}

bool FlashLog_Pulling(void) { return s_pull != PULL_IDLE; }

void FlashLog_PullTask(void)
{
    if ((s_pull != PULL_IDLE) && !UsbIo_Connected()) {      /* host gone */
        s_pull = PULL_IDLE;
        UsbIo_SetExclusive(false);
        return;
    }
    char line[32];
    switch (s_pull) {
    case PULL_IDLE:
        break;
    case PULL_HEADER:
        if (UsbIo_TxIdle()) {
            UsbIo_SetExclusive(true);         /* nothing else may interleave now */
            const int n = snprintf(line, sizeof line, "LOGDATA %lu\r\n", (unsigned long)s_pull_len);
            (void)UsbIo_WriteExcl(line, (uint32_t)n);
            s_pull = (s_pull_len > 0u) ? PULL_DATA : PULL_TRAILER;
        }
        break;
    case PULL_DATA:
        if (UsbIo_TxIdle()) {
            const uint32_t n = ((s_pull_len - s_pull_off) < PULL_CHUNK) ? (s_pull_len - s_pull_off) : PULL_CHUNK;
            const uint8_t *p = (const uint8_t *)(LOG_FLASH_BASE + s_pull_off);
            if (UsbIo_SendRaw(p, n)) {
                s_pull_crc = crc32_ieee_update(s_pull_crc, p, n);
                s_pull_off += n;
                if (s_pull_off >= s_pull_len) { s_pull = PULL_TRAILER; }
            }
        }
        break;
    case PULL_TRAILER:
        if (UsbIo_TxIdle()) {
            const int n = snprintf(line, sizeof line, "LOGEND %08lX\r\n", (unsigned long)s_pull_crc);
            (void)UsbIo_WriteExcl(line, (uint32_t)n);
            UsbIo_SetExclusive(false);
            s_pull = PULL_IDLE;
        }
        break;
    default:
        s_pull = PULL_IDLE;
        break;
    }
}
