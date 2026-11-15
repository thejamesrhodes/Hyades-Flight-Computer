/* ============================================================================
 * alloc.h  —  Canard control allocation (ALLOC-01)
 *
 * Sits between the control law and the PWM driver. The law produces four fin
 * commands (normalised, [-1, +1] = +/-FIN_DEFLECT_DEG); the allocator turns
 * them into the four commands actually sent, in whole PWM timer ticks:
 *
 *   1. Decompose into the cruciform virtual axes
 *        roll r = (f0+f1+f2+f3)/4,  pitch p = (f0-f2)/2,  yaw y = (f1-f3)/2,
 *        null n = (f0+f2-f1-f3)/4   (no net force or moment: the pairs cancel)
 *   2. Null-space dither (CTRL_DITHER_NULL): the dead-band dither goes on n,
 *      so each servo is dithered as before but the canards add no roll,
 *      pitch or yaw moment from it.
 *   3. Priority saturation: roll first, then pitch/yaw (scaled together so the
 *      direction is kept), then dither, within CTRL_FIN_LIMIT.
 *   4. Sum-preserving quantisation (CTRL_ALLOC_STAGGER): each fin is rounded
 *      to a whole tick, but the SUM is kept to the nearest tick, so the roll
 *      moment resolves to 1/4 of a tick. Extra ticks go to an opposite pair
 *      first (pure roll, the pair's forces cancel) and the odd tick alternates
 *      between the two fins of that pair frame by frame (its small pitch
 *      moment averages to zero).
 *
 * Pure module: no I/O, no globals, host-testable. Per-fin zero offsets
 * (servo spline indexing) are applied in the PWM layer in whole ticks, so
 * they never disturb the quantisation done here.
 * ============================================================================ */
#ifndef ALLOC_H_
#define ALLOC_H_

#include <stdint.h>
#include <stdbool.h>
#include "board.h"

/* PWM ticks per normalised fin unit: 30 deg x 10.34 us/deg x ticks/us
 * (310.3 at a 1 MHz tick, 2482.8 at 8 MHz: not a whole number, the quantiser
 * rounds, so full authority may be up to half a tick, 0.006 deg, over). One
 * tick is 1/ALLOC_TICKS_PER_NORM. */
#define ALLOC_US_PER_DEG     ((float)(SERVO_MAX_US - SERVO_MIN_US) / SERVO_RANGE_DEG)
#define ALLOC_TICKS_PER_NORM (FIN_DEFLECT_DEG * ALLOC_US_PER_DEG * (float)PWM_TICKS_PER_US)

typedef struct {
    uint32_t k;          /* frames since actuation began (dither phase, tie rotation) */
    float    dither;     /* null-space dither applied this frame (normalised)         */
    float    v[4];       /* virtual command after saturation: r, p, y, n              */
    int32_t  tick[4];    /* output, ticks about each fin's zero                       */
} Alloc;

void    Alloc_Init(Alloc *a);
/* cmd: law output. active: actuation enabled (mode not OFF); when false the
 * output equals cmd and the dither phase restarts. dither: add the null-space
 * dither this frame (the flight code sets it only in BOOST/COAST, so the servos
 * are not worn on the pad or bench). out: commands to send. */
void    Alloc_Apply(Alloc *a, const float cmd[4], bool active, bool dither, float out[4]);
/* Normalised command -> ticks about zero (clamped to +/-1, range-checked).
 * Shared with the PWM driver and the SITL so all three quantise identically. */
int32_t Alloc_NormToTicks(float norm);
float   Alloc_TicksToNorm(int32_t ticks);

#endif /* ALLOC_H_ */
