/* ============================================================================
 * bench.h  —  Bench profile player (BENCH-02 servo identification)
 *
 * BENCH ONLY: compiled with CFG_BENCH_PROFILE 1 (board.h), which a flight
 * build refuses. Plays a stored command profile to one or more canard servos
 * at the frame rate, with the board LEDs as a timing mark for a camera, so
 * the servo's response can be measured against a command known frame by frame.
 *
 * One run (frames; the caller steps once per control frame):
 *   PRE   BENCH_PRE_FRAMES    selected fins at 0, LEDs off
 *   RUN   profile length N    frame i: command of the profile, LEDs on while
 *                             (i / BENCH_LED_HALF) is even. N is 97 x an odd
 *                             number, different for every profile.
 *   POST  BENCH_POST_FRAMES   selected fins at 0, LEDs off
 * The caller writes the LED and the PWM compare values in the same frame,
 * before PWM_Commit, so the LED edge and the servo pulse coincide.
 *
 * A profile is a list of segments in whole PWM ticks about the fin's zero:
 *   MOVE   reach target a after b frames (0 or 1 = a step), then hold
 *   SINE   a x sin about the value at the start of the segment, period b frames
 *   TABLE  per-frame steps from table a (the flight replay)
 * with an optional dither flag (the flight dither: +/-BENCH_DITHER_TICKS,
 * square, sign by fin as the allocator's null axis). Integer only, so the
 * host replay (Tools/bench/servo_id/profiles.py) gives the same ticks; the
 * CRC-32 of the played sequence is checked against the table's at the end.
 *
 * HOLD keeps the selected fins at one angle until stopped (ruler readings).
 *
 * Pure module: no I/O, no globals beyond the generated tables. The profile
 * tables are generated (bench_profiles.c).
 * ============================================================================ */
#ifndef BENCH_H_
#define BENCH_H_

#include <stdint.h>
#include <stdbool.h>
#include "board.h"

#if CFG_BENCH_PROFILE

#define BENCH_PRE_FRAMES    300u
#define BENCH_POST_FRAMES   300u
#define BENCH_LED_HALF      97u     /* 0.485 s: not a whole number of 240 fps camera frames */
#define BENCH_DITHER_TICKS  14      /* +/-1.75 us at 8 ticks/us (CTRL_DITHER_AMP_RAD, test_bench.c) */
#define BENCH_DITHER_HALF   2u      /* frames per half period: 50 Hz (CTRL_DITHER_PERIOD_N / 2)    */
#define BENCH_TICKS_MAX     2483    /* 30 deg: the command authority (ALLOC_MAX_TICKS)             */
#define BENCH_FINS          4u
#define BENCH_MASK_ALL      0x0Fu

typedef enum { BSEG_MOVE = 0, BSEG_SINE = 1, BSEG_TABLE = 2 } BenchKind;
#define BSEG_F_DITHER       0x01u

typedef struct {
    uint8_t  kind;      /* BenchKind                                            */
    uint8_t  flags;     /* BSEG_F_*                                             */
    uint16_t frames;    /* duration, >= 1                                       */
    int16_t  a;         /* MOVE: target. SINE: amplitude. TABLE: table index    */
    uint16_t b;         /* MOVE: frames to reach the target. SINE: period       */
} BenchSeg;

typedef struct {
    const char     *name;
    const BenchSeg *seg;
    uint16_t        nseg;
    uint32_t        frames;   /* RUN length (sum of the segments)               */
    uint32_t        crc;      /* CRC-32 (zlib) of the int16 LE ticks + dither   */
} BenchProfile;

typedef enum { BENCH_IDLE = 0, BENCH_PRE, BENCH_RUN, BENCH_POST, BENCH_HOLD } BenchState;

typedef struct {
    BenchState          st;
    const BenchProfile *p;
    uint8_t   mask;     /* fins driven (bit per fin)                            */
    uint32_t  k;        /* frame within the state (RUN: profile frame index)    */
    uint32_t  iseg, j;  /* segment and frame within it                          */
    int32_t   base;     /* value when the segment began                         */
    int32_t   val;      /* last commanded value, ticks                          */
    uint32_t  crc;      /* of the frames played so far                          */
} Bench;

typedef struct {
    int32_t ticks;      /* command about zero, before dither                    */
    int32_t dither;     /* +/-BENCH_DITHER_TICKS or 0 (fins 0,2: +, fins 1,3: -) */
    uint8_t mask;
    bool    led;
    bool    done;       /* set on the last RUN frame                            */
} BenchOut;

extern const int16_t        k_bench_sin_q[257];
extern const int8_t *const  k_bench_tables[];
extern const uint16_t       k_bench_table_len[];
extern const uint32_t       k_bench_ntables;
extern const BenchProfile   k_bench_profiles[];
extern const uint32_t       k_bench_nprofiles;

void Bench_Init(Bench *b);
static inline bool Bench_Active(const Bench *b) { return b->st != BENCH_IDLE; }
/* Start a run (from any state). False: no profile or no fin selected. */
bool Bench_Start(Bench *b, const BenchProfile *p, uint8_t mask);
/* Hold the selected fins at ticks (clamped to +/-BENCH_TICKS_MAX) until stopped. */
void Bench_Hold(Bench *b, uint8_t mask, int32_t ticks);
void Bench_Stop(Bench *b);
/* One control frame while active: what to command and show this frame. */
void Bench_Step(Bench *b, BenchOut *o);
/* Ticks about zero for one fin this frame (0 for a fin that is not selected). */
int32_t Bench_FinTicks(const BenchOut *o, uint32_t fin);
/* Profile by name or by decimal id; NULL if there is none. */
const BenchProfile *Bench_Find(const char *s);
/* Millidegrees at the canard -> ticks, rounded (16 ticks per 225 mdeg). */
int32_t Bench_MdegToTicks(int32_t mdeg);

#endif /* CFG_BENCH_PROFILE */
#endif /* BENCH_H_ */
