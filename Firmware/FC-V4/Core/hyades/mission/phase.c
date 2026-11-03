/* ============================================================================
 * phase.c  —  Flight phase state machine (see phase.h)
 * ============================================================================ */
#include "phase.h"

static const Phase k_phases[PHASE_COUNT] = {
    PH_BOOT, PH_CALIBRATE, PH_SETTLING, PH_READY, PH_BOOST,
    PH_COAST, PH_DESCENT, PH_LANDED, PH_LOCKOUT, PH_CATO_SAFE
};
static const char *const k_names[PHASE_COUNT] = {
    "BOOT", "CAL", "SETTLE", "READY", "BOOST",
    "COAST", "DESCENT", "LANDED", "LOCKOUT", "CATO"
};

/* Allowed transitions: k_allowed[from] is a bit mask over target indices. */
#define B(p) (1u << (p))
enum { I_BOOT, I_CAL, I_SETTLE, I_READY, I_BOOST, I_COAST, I_DESC, I_LANDED, I_LOCK, I_CATO };
static const uint16_t k_allowed[PHASE_COUNT] = {
    [I_BOOT]   = B(I_CAL)   | B(I_LOCK),
    [I_CAL]    = B(I_SETTLE)| B(I_LOCK),
    [I_SETTLE] = B(I_READY) | B(I_LOCK),
    [I_READY]  = B(I_SETTLE)| B(I_BOOST) | B(I_LOCK),
    [I_BOOST]  = B(I_COAST) | B(I_CATO),
    [I_COAST]  = B(I_DESC)  | B(I_CATO),
    [I_DESC]   = B(I_LANDED),
    [I_LANDED] = 0u,
    [I_LOCK]   = 0u,
    [I_CATO]   = B(I_LANDED),
};
#undef B

static void store(PhaseWord *w, Phase p) { w->v = (uint32_t)p; w->inv = ~(uint32_t)p; }

uint8_t Phase_Index(Phase p)
{
    for (uint8_t i = 0u; i < PHASE_COUNT; i++) { if (k_phases[i] == p) { return i; } }
    return 0xFFu;
}

const char *Phase_Name(Phase p)
{
    const uint8_t i = Phase_Index(p);
    return (i < PHASE_COUNT) ? k_names[i] : "INVALID";
}

void Phase_Init(PhaseWord *w) { store(w, PH_BOOT); }

bool Phase_Read(const PhaseWord *w, Phase *out)
{
    if ((w->v ^ w->inv) != 0xFFFFFFFFu) { return false; }
    if (Phase_Index((Phase)w->v) >= PHASE_COUNT) { return false; }
    *out = (Phase)w->v;
    return true;
}

bool Phase_Transition(PhaseWord *w, Phase to)
{
    Phase from;
    if (!Phase_Read(w, &from)) { return false; }
    const uint8_t fi = Phase_Index(from), ti = Phase_Index(to);
    if (ti >= PHASE_COUNT) { return false; }
    if ((k_allowed[fi] & (1u << ti)) == 0u) { return false; }
    store(w, to);
    return true;
}

bool Phase_Recover(PhaseWord *w)
{
    Phase from;
    if (!Phase_Read(w, &from)) { from = PH_LOCKOUT; }   /* corrupt word -> pad-safe anyway */
    if (from != PH_LOCKOUT) { return false; }
    store(w, PH_SETTLING);
    return true;
}

void Phase_ForceSafe(PhaseWord *w, bool in_flight)
{
    store(w, in_flight ? PH_CATO_SAFE : PH_LOCKOUT);
}

bool Phase_InFlight(Phase p)
{
    return (p == PH_BOOST) || (p == PH_COAST) || (p == PH_DESCENT);
}

bool Phase_ActuationAllowed(Phase p, bool bench_ready)
{
    return (p == PH_BOOST) || (p == PH_COAST) || ((p == PH_READY) && bench_ready);
}
