/* ============================================================================
 * sched.h  —  Cyclic-executive timebase
 *
 * Provides a 32-bit, 1 MHz, free-running microsecond clock from TIM5 and the
 * primitives the superloop in main.c uses to pace fixed-rate frames and detect
 * overruns. There is no preemption between frame tasks, so no task-to-task
 * locking is required (only the ISR/frame boundaries need protection).
 * ============================================================================ */
#ifndef SCHED_H
#define SCHED_H

#include <stdint.h>

/* Configure TIM5 as a 1 MHz, 32-bit, free-running up-counter. Must be called
 * after the clock tree is up and before the superloop starts. */
void     Sched_Init(void);

/* Free-running microsecond counter (wraps at 2^32 us ~= 71.6 min). All frame
 * timing uses signed-delta comparisons so a single wrap is handled correctly
 * within any horizon shorter than ~35 minutes. */
static inline uint32_t Sched_MicrosNow(void);

/* Busy-wait until the (absolute) deadline is reached. Bounded by the timer. */
void     Sched_WaitUntil(uint32_t deadline_us);

#include "stm32f4xx_hal.h"
static inline uint32_t Sched_MicrosNow(void) { return TIM5->CNT; }

#endif /* SCHED_H */
