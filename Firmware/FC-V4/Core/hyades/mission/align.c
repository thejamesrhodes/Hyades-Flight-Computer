/* ============================================================================
 * align.c  —  see align.h
 * ============================================================================ */
#include "align.h"

_Static_assert(FINCAL_LIMIT_MDEG == 56000, "AL_ERR_RANGE text below quotes the limit");

void Align_Init(Align *a)
{
    a->active = false;
    a->last_end = AL_END_NONE;
    for (uint32_t i = 0u; i < FINCAL_FINS; i++) { a->base[i] = 0; a->trial[i] = 0; }
}

bool Align_CanBegin(Phase ph)
{
    return (ph == PH_SETTLING) || (ph == PH_READY);
}

AlignResult Align_Begin(Align *a, const int32_t stored[FINCAL_FINS], Phase ph, bool hard_fault)
{
    if (a->active) { return AL_ERR_ACTIVE; }
    if (!Align_CanBegin(ph)) { return AL_ERR_PHASE; }
    if (hard_fault) { return AL_ERR_FAULT; }
    for (uint32_t i = 0u; i < FINCAL_FINS; i++) { a->base[i] = stored[i]; a->trial[i] = stored[i]; }
    a->active = true;
    a->last_end = AL_END_NONE;
    return AL_OK;
}

static AlignResult put(Align *a, uint32_t fin, int32_t v)
{
    if ((v > FINCAL_LIMIT_MDEG) || (v < -FINCAL_LIMIT_MDEG)) { return AL_ERR_RANGE; }
    a->trial[fin] = v;
    return AL_OK;
}

AlignResult Align_Jog(Align *a, uint32_t fin, int32_t d_mdeg)
{
    if (!a->active) { return AL_ERR_IDLE; }
    if (fin >= FINCAL_FINS) { return AL_ERR_FIN; }
    if ((d_mdeg > ALIGN_STEP_MAX_MDEG) || (d_mdeg < -ALIGN_STEP_MAX_MDEG)) { return AL_ERR_STEP; }
    return put(a, fin, a->trial[fin] + d_mdeg);         /* |sum| <= 100 000: no overflow */
}

AlignResult Align_Set(Align *a, uint32_t fin, int32_t mdeg)
{
    if (!a->active) { return AL_ERR_IDLE; }
    if (fin >= FINCAL_FINS) { return AL_ERR_FIN; }
    return put(a, fin, mdeg);
}

const int32_t *Align_End(Align *a, AlignEnd why)
{
    const bool keep = a->active && (why == AL_END_SAVED);
    a->active = false;
    a->last_end = why;
    if (!keep) { for (uint32_t i = 0u; i < FINCAL_FINS; i++) { a->trial[i] = a->base[i]; } }
    return keep ? a->trial : a->base;
}

AlignEnd Align_Poll(Align *a, Phase ph)
{
    if (!a->active || (ph == PH_ALIGN)) { return AL_END_NONE; }
    AlignEnd why = AL_END_PHASE;
    if ((ph == PH_LOCKOUT) || (ph == PH_CATO_SAFE)) { why = AL_END_FAULT; }
    else if (ph == PH_BOOST)                         { why = AL_END_LIFTOFF; }
    else                                             { /* any other exit */ }
    (void)Align_End(a, why);
    return why;
}

const int32_t *Align_Zero(const Align *a, const int32_t stored[FINCAL_FINS])
{
    return a->active ? a->trial : stored;
}

const char *Align_ResultText(AlignResult r)
{
    switch (r) {
    case AL_OK:         return "OK";
    case AL_ERR_PHASE:  return "only on the pad (SETTLING or READY)";
    case AL_ERR_FAULT:  return "a HARD fault is active";
    case AL_ERR_ACTIVE: return "session already active";
    case AL_ERR_IDLE:   return "no session ('cal begin' first)";
    case AL_ERR_FIN:    return "fin must be 0..3";
    case AL_ERR_RANGE:  return "zero outside +/-56 deg (the servo travel from neutral)";
    case AL_ERR_STEP:   return "jog larger than 10 deg (use 'cal set')";
    default:            return "?";
    }
}

const char *Align_EndText(AlignEnd e)
{
    switch (e) {
    case AL_END_NONE:    return "none";
    case AL_END_SAVED:   return "saved";
    case AL_END_CANCEL:  return "cancelled";
    case AL_END_FAULT:   return "HARD fault";
    case AL_END_LIFTOFF: return "liftoff";
    case AL_END_PHASE:   return "phase change";
    default:             return "?";
    }
}

bool Align_ParseMdeg(const char *s, int32_t *mdeg)
{
    if (s == 0) { return false; }
    int32_t sign = 1;
    uint32_t i = 0u;
    if ((s[0] == '-') || (s[0] == '+')) { sign = (s[0] == '-') ? -1 : 1; i = 1u; }
    int32_t whole = 0, frac = 0;
    uint32_t nd_whole = 0u, nd_frac = 0u;
    bool dot = false;
    for (; (s[i] != '\0') && (i < 12u); i++) {
        const char c = s[i];
        if ((c == '.') && !dot) { dot = true; continue; }
        if ((c < '0') || (c > '9')) { return false; }
        const int32_t d = (int32_t)(c - '0');
        if (!dot) {
            if (nd_whole >= 2u) { return false; }       /* |value| < 100 deg */
            whole = (whole * 10) + d; nd_whole++;
        } else {
            if (nd_frac >= 3u) { return false; }        /* millidegree resolution */
            frac = (frac * 10) + d; nd_frac++;
        }
    }
    if ((s[i] != '\0') || ((nd_whole + nd_frac) == 0u)) { return false; }
    for (uint32_t k = nd_frac; k < 3u; k++) { frac *= 10; }
    *mdeg = sign * ((whole * 1000) + frac);
    return true;
}
