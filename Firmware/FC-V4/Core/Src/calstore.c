/* ============================================================================
 * calstore.c  —  see calstore.h. Register-level flash programming (RM0383 §3).
 * ============================================================================ */
#include "calstore.h"
#include "board.h"
#include "main.h"
#include <string.h>

#define SR_ERRORS   (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR)
#define BSY_SPIN    20000u          /* >> 16 us word program at 96 MHz */

typedef char cal_region_check[((CAL_FLASH_SIZE % FINCAL_REC_SIZE) == 0u)
                              && ((CAL_FLASH_BASE + CAL_FLASH_SIZE) == LOG_FLASH_BASE) ? 1 : -1];

static bool wait_bsy(void)
{
    for (uint32_t i = 0u; i < BSY_SPIN; i++) {
        if ((FLASH->SR & FLASH_SR_BSY) == 0u) { return (FLASH->SR & SR_ERRORS) == 0u; }
    }
    return false;
}

/* Program one record. The flight log may hold the flash unlocked while it is
 * recording (flashlog.c); restore whatever lock state we found. */
static bool program(uint32_t slot, const uint8_t rec[FINCAL_REC_SIZE])
{
    const bool was_locked = (FLASH->CR & FLASH_CR_LOCK) != 0u;
    if (was_locked) { FLASH->KEYR = 0x45670123u; FLASH->KEYR = 0xCDEF89ABu; }
    FLASH->SR = FLASH_SR_EOP | SR_ERRORS;
    volatile uint32_t *dst = (volatile uint32_t *)(CAL_FLASH_BASE + (slot * FINCAL_REC_SIZE));
    bool ok = true;
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
    for (uint32_t w = 0u; (w < (FINCAL_REC_SIZE / 4u)) && ok; w++) {
        uint32_t v;
        memcpy(&v, &rec[w * 4u], 4u);
        dst[w] = v;
        __DSB();
        ok = wait_bsy();
    }
    FLASH->CR &= ~FLASH_CR_PG;
    if (!ok) { FLASH->SR = SR_ERRORS; }
    if (was_locked) { FLASH->CR = FLASH_CR_LOCK; }
    /* The scan may have cached this slot as erased: reset the ART data cache
     * so the read-back verify sees the array. */
    FLASH->ACR &= ~FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |= FLASH_ACR_DCEN;
    return ok;
}

void CalStore_Init(CalStore *c)
{
    c->write_errors = 0u;
    FinCal_Scan((const uint8_t *)CAL_FLASH_BASE, CAL_FLASH_SIZE, &c->s);
}

bool CalStore_Full(const CalStore *c) { return c->s.next >= c->s.slots; }

static bool append(CalStore *c, uint16_t seq, const int16_t off_mdeg[FINCAL_FINS])
{
    if (!FinCal_InRange(off_mdeg) || CalStore_Full(c)) { return false; }
    uint8_t rec[FINCAL_REC_SIZE];
    FinCal_Pack(rec, seq, off_mdeg);
    const uint32_t slot = c->s.next;
    const bool ok = program(slot, rec);
    c->s.next = slot + 1u;                        /* a torn slot is skipped, never rewritten */
    FinCal back;                                   /* trust the array, not the RAM copy */
    if (ok && FinCal_Unpack((const uint8_t *)(CAL_FLASH_BASE + (slot * FINCAL_REC_SIZE)), &back)) {
        c->s.cal = back;
        return true;
    }
    c->write_errors++;
    c->s.bad++;
    return false;
}

bool CalStore_Save(CalStore *c, const int16_t off_mdeg[FINCAL_FINS])
{
    const uint16_t seq = c->s.cal.valid ? (uint16_t)(c->s.cal.seq + 1u) : 1u;
    return append(c, seq, off_mdeg);
}

bool CalStore_Rewrite(CalStore *c)
{
    const FinCal keep = c->s.cal;
    FinCal_Scan((const uint8_t *)CAL_FLASH_BASE, CAL_FLASH_SIZE, &c->s);    /* now erased */
    if (!keep.valid) { return true; }
    return append(c, keep.seq, keep.off_mdeg);
}
