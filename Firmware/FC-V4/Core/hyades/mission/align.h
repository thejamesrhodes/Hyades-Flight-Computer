/* ============================================================================
 * align.h  —  Canard zero alignment session (CAL-01)
 *
 * On the pad, before READY, the operator jogs each canard's zero offset until
 * the canard sits on its jig or laser reference, then saves. The session never
 * touches the control path: it only edits the per-fin zero that the PWM layer
 * adds to every command. Before READY the flight loop commands every fin to 0,
 * so the canard sits exactly on the trial zero being jogged.
 *
 * Safety (all enforced here, host-tested):
 *  - begins only in BOOT / CALIBRATE / SETTLING with no HARD fault active;
 *  - while active the app holds READY off (FlightIn.ready_inhibit);
 *  - ends, reverting to the stored zero, on: cancel, ALIGN_IDLE_MS without a
 *    command, any phase outside the allowed set, a HARD fault, or the USB
 *    link dropping. Only an explicit save keeps the trial values;
 *  - every value stays within +/-FINCAL_LIMIT_MDEG, every jog within
 *    +/-ALIGN_STEP_MAX_MDEG: out-of-range input is refused, never clamped.
 *
 * Pure module (no I/O, no globals). Angles are integer millidegrees.
 * ============================================================================ */
#ifndef ALIGN_H_
#define ALIGN_H_

#include <stdint.h>
#include <stdbool.h>
#include "phase.h"
#include "fincal.h"

#define ALIGN_IDLE_MS         60000u   /* revert after 60 s without a command */
#define ALIGN_STEP_MAX_MDEG   2000     /* one jog moves a canard at most 2 deg */

typedef enum {
    AL_OK = 0,
    AL_ERR_PHASE,     /* not on the pad before READY       */
    AL_ERR_FAULT,     /* a HARD fault is active            */
    AL_ERR_ACTIVE,    /* a session is already running      */
    AL_ERR_IDLE,      /* no session                        */
    AL_ERR_FIN,       /* fin index outside 0..3            */
    AL_ERR_RANGE,     /* result outside +/-limit           */
    AL_ERR_STEP,      /* single jog larger than the step   */
} AlignResult;

typedef enum {
    AL_END_NONE = 0, AL_END_SAVED, AL_END_CANCEL, AL_END_TIMEOUT, AL_END_PHASE, AL_END_FAULT, AL_END_LINK
} AlignEnd;

typedef struct {
    bool     active;
    int16_t  base[FINCAL_FINS];    /* stored zero when the session began    */
    int16_t  trial[FINCAL_FINS];   /* zero being jogged                     */
    uint32_t last_ms;              /* last accepted command                 */
    AlignEnd last_end;
} Align;

void        Align_Init(Align *a);
static inline bool Align_Active(const Align *a) { return a->active; }
bool        Align_PhaseAllowed(Phase ph);
AlignResult Align_Begin(Align *a, const int16_t stored[FINCAL_FINS], Phase ph, bool hard_fault, uint32_t now_ms);
AlignResult Align_Jog(Align *a, uint32_t fin, int32_t d_mdeg, uint32_t now_ms);
AlignResult Align_Set(Align *a, uint32_t fin, int32_t mdeg, uint32_t now_ms);
/* End now. Returns the zero to apply: trial for AL_END_SAVED, base otherwise. */
const int16_t *Align_End(Align *a, AlignEnd why);
/* Every frame. Ends the session (reverting) on timeout, phase, fault or link
 * loss and returns the reason; AL_END_NONE otherwise. */
AlignEnd    Align_Poll(Align *a, Phase ph, bool hard_fault, bool link_ok, uint32_t now_ms);
/* The zero the PWM layer should be using now (trial while active). */
const int16_t *Align_Zero(const Align *a, const int16_t stored[FINCAL_FINS]);
uint32_t    Align_IdleLeftMs(const Align *a, uint32_t now_ms);
const char *Align_ResultText(AlignResult r);
const char *Align_EndText(AlignEnd e);
/* Decimal degrees ("-0.125", "+2", ".5") -> millidegrees, at most 3
 * decimals, |value| <= 99.999. No float parsing. */
bool        Align_ParseMdeg(const char *s, int32_t *mdeg);

#endif /* ALIGN_H_ */
