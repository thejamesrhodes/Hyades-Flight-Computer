/* ============================================================================
 * align.h  —  Canard zero alignment session (CAL-01)
 *
 * The operator jogs each canard's zero until the canard sits on its jig or
 * laser reference, then saves. The session is a phase of its own, ALIGN
 * (phase.c): it can be entered from SETTLING or READY, and it is left only by
 * save or cancel (both to SETTLING, so READY needs the full hold again), a
 * HARD fault (LOCKOUT) or a liftoff (BOOST, sequenced as from SETTLING). It
 * has no timeout: the phase itself is the protection, because ALIGN can never
 * actuate and never reach READY.
 *
 * The session never touches the control path. It only edits the per-fin zero
 * that the PWM layer adds to every command; in ALIGN the flight loop commands
 * every fin to 0, so each canard sits exactly on its trial zero. Leaving ALIGN
 * any way but save restores the stored zero.
 *
 * Limits: a zero within +/-FINCAL_LIMIT_MDEG (the servo's half range, so any
 * horn indexing can be corrected) and one jog within +/-ALIGN_STEP_MAX_MDEG.
 * Out-of-range input is refused, never clamped.
 *
 * Pure module (no I/O, no globals). Angles are integer millidegrees.
 * ============================================================================ */
#ifndef ALIGN_H_
#define ALIGN_H_

#include <stdint.h>
#include <stdbool.h>
#include "phase.h"
#include "fincal.h"

#define ALIGN_STEP_MAX_MDEG   10000    /* one jog moves a canard at most 10 deg */

typedef enum {
    AL_OK = 0,
    AL_ERR_PHASE,     /* not in SETTLING or READY             */
    AL_ERR_FAULT,     /* a HARD fault is active               */
    AL_ERR_ACTIVE,    /* a session is already running         */
    AL_ERR_IDLE,      /* no session                           */
    AL_ERR_FIN,       /* fin index outside 0..3               */
    AL_ERR_RANGE,     /* result outside +/-limit              */
    AL_ERR_STEP,      /* single jog larger than the step      */
} AlignResult;

typedef enum {
    AL_END_NONE = 0, AL_END_SAVED, AL_END_CANCEL, AL_END_FAULT, AL_END_LIFTOFF, AL_END_PHASE
} AlignEnd;

typedef struct {
    bool     active;
    int32_t  base[FINCAL_FINS];    /* stored zero when the session began */
    int32_t  trial[FINCAL_FINS];   /* zero being jogged                  */
    AlignEnd last_end;
} Align;

void        Align_Init(Align *a);
static inline bool Align_Active(const Align *a) { return a->active; }
bool        Align_CanBegin(Phase ph);            /* SETTLING or READY */
AlignResult Align_Begin(Align *a, const int32_t stored[FINCAL_FINS], Phase ph, bool hard_fault);
AlignResult Align_Jog(Align *a, uint32_t fin, int32_t d_mdeg);
AlignResult Align_Set(Align *a, uint32_t fin, int32_t mdeg);
/* End now. Returns the zero to apply: trial for AL_END_SAVED, base otherwise. */
const int32_t *Align_End(Align *a, AlignEnd why);
/* Every frame, after the phase machine has run: if the phase is no longer
 * ALIGN, the session ends (reverting) and the reason is returned. */
AlignEnd    Align_Poll(Align *a, Phase ph);
/* The zero the PWM layer should be using now (trial while active). */
const int32_t *Align_Zero(const Align *a, const int32_t stored[FINCAL_FINS]);
const char *Align_ResultText(AlignResult r);
const char *Align_EndText(AlignEnd e);
/* Decimal degrees ("-0.125", "+45", ".5") -> millidegrees, at most 3
 * decimals, |value| <= 99.999. No float parsing. */
bool        Align_ParseMdeg(const char *s, int32_t *mdeg);

#endif /* ALIGN_H_ */
