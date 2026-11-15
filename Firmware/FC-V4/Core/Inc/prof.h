/* ============================================================================
 * prof.h  --  cycle-accurate execution-time high-water marks (target only)
 *
 * DWT->CYCCNT counts core clocks (HCLK, 96 MHz on FC-V4; wraps every 44.7 s,
 * so any single difference below that is exact). Measured times include any
 * interrupt that pre-empts the code: they are response times as the frame
 * sees them, not isolated execution times.
 *
 * Prof_Mark(0) starts Flight_Step; Prof_Mark(k) closes section k-1:
 *   0 estimate (IEKF + NAV), 1 monitor (FDIR, detectors, phase, mode),
 *   2 control (q schedule, roll test, roll law incl. observer), 3 alloc.
 * flight.c calls it through FLIGHT_PROF_MARK, which is empty on the host.
 * ============================================================================ */
#ifndef PROF_H_
#define PROF_H_

#include <stdint.h>
#include "stm32f4xx.h"

#define PROF_N_SEC 4u

typedef struct { uint32_t last, max; } ProfSec;

extern ProfSec g_prof_sec[PROF_N_SEC];
extern const char *const g_prof_sec_name[PROF_N_SEC];

void Prof_Init(void);                    /* enable the cycle counter          */
void Prof_Reset(void);                   /* clear the section high-water marks */
void Prof_Mark(uint32_t id);

static inline uint32_t Prof_Cyc(void) { return DWT->CYCCNT; }

#endif /* PROF_H_ */
