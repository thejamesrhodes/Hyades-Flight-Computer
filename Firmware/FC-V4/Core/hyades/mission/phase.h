/* ============================================================================
 * phase.h  —  Flight phase state machine (FDS-01 §3.1), protected encoding
 *
 *   BOOT -> CALIBRATE -> SETTLING <-> READY -> BOOST -> COAST -> DESCENT -> LANDED
 *     |         |            |          |        |        |
 *     +---------+------------+----------+-> LOCKOUT       +-> CATO_SAFE -> LANDED
 *                                                (BOOST/COAST only)
 *
 * - Transitions are table-driven and one-way except the pad demotion
 *   READY -> SETTLING. Anything not in the table is refused (state unchanged).
 * - Phase codes are 16-bit values with pairwise Hamming distance >= 6, so no
 *   1..5-bit upset turns one valid phase into another. The word is stored with
 *   its complement; a mismatch reads as corrupt and the caller must go safe.
 * - LOCKOUT and LANDED are terminal. Leaving LOCKOUT needs a reset (human).
 *
 * Detectors for liftoff/burnout/apogee/landing are not implemented yet: the
 * flight transitions exist and are unit-tested, but nothing requests them.
 *
 * Pure module: no I/O, no globals.
 * ============================================================================ */
#ifndef PHASE_H_
#define PHASE_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    PH_BOOT      = 0xF0B8u,
    PH_CALIBRATE = 0xB579u,
    PH_SETTLING  = 0x6A19u,
    PH_READY     = 0x7847u,
    PH_BOOST     = 0x15C7u,
    PH_COAST     = 0x7C92u,
    PH_DESCENT   = 0x415Bu,
    PH_LANDED    = 0xEEC5u,
    PH_LOCKOUT   = 0x1B9Fu,
    PH_CATO_SAFE = 0xD32Cu
} Phase;

#define PHASE_COUNT 10u

typedef struct { uint32_t v; uint32_t inv; } PhaseWord;   /* value + ~value */

void        Phase_Init(PhaseWord *w);                        /* -> PH_BOOT      */
bool        Phase_Read(const PhaseWord *w, Phase *out);      /* false: corrupt  */
bool        Phase_Transition(PhaseWord *w, Phase to);        /* false: refused  */

/* Safe-state entry used when the word itself is found corrupt: always
 * succeeds, landing in LOCKOUT (pad) or CATO_SAFE (in_flight). */
void        Phase_ForceSafe(PhaseWord *w, bool in_flight);

uint8_t     Phase_Index(Phase p);        /* 0..PHASE_COUNT-1, 0xFF if invalid  */
const char *Phase_Name(Phase p);
bool        Phase_InFlight(Phase p);     /* BOOST, COAST, DESCENT              */

/* Fins may move only in BOOST/COAST, and in READY when bench_ready is set
 * (bench/HIL testing with the airframe on the ground). */
bool        Phase_ActuationAllowed(Phase p, bool bench_ready);

#endif /* PHASE_H_ */
