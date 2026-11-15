/* ============================================================================
 * rolltest.h  —  In-flight roll test manoeuvre (SYSID-01 section 6)
 *
 * Once per flight, in COAST: wait RT_T0_S after COAST entry; start only if the
 * control is FULL, there is no fault and q_hat >= RT_Q_START_PA (one attempt:
 * q only falls in coast). Then
 *   EXC     a 3-2-1-1 added at the canard: +a (3 units), -a (2), +a (1), -a (1),
 *           unit RT_UNIT_S, a = RT_UA_DEG x RT_Q_REF_PA / q_hat at the start
 *           (latched, so the log reconstructs it exactly), at most RT_UA_MAX_DEG;
 *   GAP     RT_GAP_S of the zero-rate law;
 *   DOUBLET roll-rate reference +RT_RATE_DPS for RT_HOLD_S, then -RT_RATE_DPS
 *           for RT_HOLD_S, then 0.
 * Abandoned at once (outputs 0, the zero-rate law resumes) if q_hat <
 * RT_Q_ABORT_PA, |p| > RT_P_ABORT_DPS, the control is not FULL, a fault, or
 * the phase leaves COAST. Pure module: the caller feeds the outputs to the
 * roll law (Controller p_ref, u_exc) and logs the events.
 * ============================================================================ */
#ifndef ROLLTEST_H_
#define ROLLTEST_H_

#include <stdint.h>
#include <stdbool.h>
#include "phase.h"

typedef enum { RT_IDLE = 0, RT_WAIT, RT_EXC, RT_GAP, RT_DOUBLET, RT_DONE, RT_SKIPPED, RT_ABORTED } RtState;
typedef enum { RT_EV_NONE = 0, RT_EV_START, RT_EV_DONE, RT_EV_SKIP, RT_EV_ABORT } RtEvent;
typedef enum { RT_WHY_NONE = 0, RT_WHY_Q, RT_WHY_RATE, RT_WHY_FAULT, RT_WHY_PHASE } RtWhy;

typedef struct {
    RtState  st;
    RtState  st_end;    /* stage reached when it ended (EXC, GAP or DOUBLET)  */
    uint32_t k;         /* frames in the current state                     */
    uint32_t n0, nu, ng, nh;   /* frames: wait, unit, gap, hold            */
    float    ua;        /* latched canard amplitude, rad                     */
    float    q_start;   /* q_hat at the start, Pa                            */
    float    p_ref;     /* outputs, rad/s and rad at the canard              */
    float    u_exc;
    RtWhy    why;       /* reason for the last skip or abort                 */
} RollTest;

void    RollTest_Init(RollTest *r, float frame_hz);
/* ok: control FULL and no fault. p: roll rate, rad/s. Returns the event of this frame. */
RtEvent RollTest_Step(RollTest *r, Phase ph, bool ok, bool q_valid, float qbar, float p);
bool    RollTest_Active(const RollTest *r);
const char *RollTest_Why(RtWhy w);

#endif /* ROLLTEST_H_ */
