/* prof.c -- see prof.h */
#include "prof.h"

ProfSec g_prof_sec[PROF_N_SEC];
const char *const g_prof_sec_name[PROF_N_SEC] = { "estimate", "monitor", "control", "alloc" };
static uint32_t s_prev;

void Prof_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    Prof_Reset();
}

void Prof_Reset(void)
{
    for (uint32_t i = 0u; i < PROF_N_SEC; i++) { g_prof_sec[i].last = 0u; g_prof_sec[i].max = 0u; }
}

void Prof_Mark(uint32_t id)
{
    const uint32_t now = DWT->CYCCNT;
    if ((id > 0u) && (id <= PROF_N_SEC)) {
        const uint32_t d = now - s_prev;
        g_prof_sec[id - 1u].last = d;
        if (d > g_prof_sec[id - 1u].max) { g_prof_sec[id - 1u].max = d; }
    }
    s_prev = now;
}
