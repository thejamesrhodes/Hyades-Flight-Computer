/* ============================================================================
 * wdg.c  —  see wdg.h. RM0383 §17 (IWDG).
 * ============================================================================ */
#include "wdg.h"
#include "board.h"
#include "main.h"

#define KR_KICK    0xAAAAu
#define KR_UNLOCK  0x5555u
#define KR_START   0xCCCCu
#define PR_DIV4    0u          /* LSI/4   -> 125 us per count at 32 kHz */
#define PR_DIV256  6u          /* LSI/256 -> 8 ms per count             */
#define RLR_NORMAL 800u        /* 800 x 125 us = 100 ms                  */

static void wait_update(void)
{
    /* PVU/RVU clear within ~5 LSI cycles (~150 us); bounded spin. */
    for (uint32_t i = 0u; (i < 100000u) && ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0u); i++) { }
}

static void configure(uint32_t pr, uint32_t rlr)
{
    wait_update();
    IWDG->KR  = KR_UNLOCK;
    IWDG->PR  = pr;
    IWDG->RLR = rlr;
    wait_update();
    IWDG->KR  = KR_KICK;
}

void Wdg_Start(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;   /* pause under the debugger */
    IWDG->KR = KR_START;
    configure(PR_DIV4, RLR_NORMAL);
}

void Wdg_Kick(void)    { IWDG->KR = KR_KICK; }
void Wdg_Stretch(void) { configure(PR_DIV256, WDG_LONG_RELOAD); }
void Wdg_Restore(void) { configure(PR_DIV4, RLR_NORMAL); }
