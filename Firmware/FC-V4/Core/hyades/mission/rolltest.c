/* ============================================================================
 * rolltest.c  —  see rolltest.h
 * ============================================================================ */
#include "rolltest.h"
#include "board.h"
#include <math.h>

#define RT_D2R 0.017453292519943295f

static uint32_t frames(float s, float hz) { return (uint32_t)lroundf(s * hz); }

void RollTest_Init(RollTest *r, float frame_hz)
{
    r->st = RT_IDLE; r->st_end = RT_IDLE; r->k = 0u; r->ua = 0.0f; r->q_start = 0.0f; r->p_ref = 0.0f; r->u_exc = 0.0f; r->why = RT_WHY_NONE;
    r->n0 = frames(RT_T0_S, frame_hz); r->nu = frames(RT_UNIT_S, frame_hz);
    r->ng = frames(RT_GAP_S, frame_hz); r->nh = frames(RT_HOLD_S, frame_hz);
    if (r->nu == 0u) { r->nu = 1u; }                     /* never divide by zero, whatever the frame rate */
}

bool RollTest_Active(const RollTest *r) { return (r->st == RT_EXC) || (r->st == RT_GAP) || (r->st == RT_DOUBLET); }

const char *RollTest_Why(RtWhy w)
{
    switch (w) {
    case RT_WHY_Q:     return "q";
    case RT_WHY_RATE:  return "rate";
    case RT_WHY_FAULT: return "fault";
    case RT_WHY_PHASE: return "phase";
    default:           return "-";
    }
}

static RtEvent stop(RollTest *r, RtState st, RtWhy why, RtEvent ev)
{
    r->st_end = r->st;                                    /* how far it got, for the event log */
    r->st = st; r->why = why; r->k = 0u; r->p_ref = 0.0f; r->u_exc = 0.0f;
    return ev;
}

RtEvent RollTest_Step(RollTest *r, Phase ph, bool ok, bool q_valid, float qbar, float p)
{
    const bool q_ok = q_valid && isfinite(qbar);
    if (RollTest_Active(r)) {
        RtWhy why = RT_WHY_NONE;
        if (ph != PH_COAST)                                   { why = RT_WHY_PHASE; }
        else if (!ok)                                         { why = RT_WHY_FAULT; }
        else if (!q_ok || (qbar < RT_Q_ABORT_PA))             { why = RT_WHY_Q; }
        else if (!isfinite(p) || (fabsf(p) > (RT_P_ABORT_DPS * RT_D2R))) { why = RT_WHY_RATE; }
        else { /* carry on */ }
        if (why != RT_WHY_NONE) { return stop(r, RT_ABORTED, why, RT_EV_ABORT); }
    }
    RtEvent ev = RT_EV_NONE;
    switch (r->st) {
    case RT_IDLE:
        if (ph == PH_COAST) { r->st = RT_WAIT; r->k = 0u; }
        break;
    case RT_WAIT:
        if (ph != PH_COAST) { ev = stop(r, RT_SKIPPED, RT_WHY_PHASE, RT_EV_SKIP); break; }
        r->k++;
        if (r->k >= r->n0) {
            if (!ok)                                       { ev = stop(r, RT_SKIPPED, RT_WHY_FAULT, RT_EV_SKIP); }
            else if (!q_ok || (qbar < RT_Q_START_PA))      { ev = stop(r, RT_SKIPPED, RT_WHY_Q, RT_EV_SKIP); }
            else {
                r->q_start = qbar;
                r->ua = fminf(RT_UA_DEG * (RT_Q_REF_PA / qbar), RT_UA_MAX_DEG) * RT_D2R;
                r->st = RT_EXC; r->k = 0u; ev = RT_EV_START;
            }
        }
        break;
    default: break;
    }
    /* outputs of the active states (the step that starts EXC also outputs its first frame) */
    r->p_ref = 0.0f; r->u_exc = 0.0f;
    if (r->st == RT_EXC) {
        const uint32_t u = r->k / r->nu;                      /* unit index 0..6 */
        r->u_exc = ((u < 3u) || (u == 5u)) ? r->ua : -r->ua;
        r->k++;
        if (r->k >= (7u * r->nu)) { r->st = RT_GAP; r->k = 0u; }
    } else if (r->st == RT_GAP) {
        r->k++;
        if (r->k >= r->ng) { r->st = RT_DOUBLET; r->k = 0u; }
    } else if (r->st == RT_DOUBLET) {
        r->p_ref = ((r->k < r->nh) ? RT_RATE_DPS : -RT_RATE_DPS) * RT_D2R;
        r->k++;
        if (r->k >= (2u * r->nh)) { r->st_end = RT_DOUBLET; r->st = RT_DONE; r->k = 0u; ev = RT_EV_DONE; }
    } else { /* inactive: zero */ }
    return ev;
}
