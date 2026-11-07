/* ============================================================================
 * detect.h  —  Flight-event detectors: liftoff, burnout, apogee, landing
 *
 * Requests the next flight phase; the arbiter (task_fdir) applies it through
 * Phase_Transition, so the table's one-way rules still decide. Each detector
 * is a persistence counter on a threshold well separated from the pad/noise
 * level. This is a SIMPLIFICATION of the FDS-01 §5.3 SPRT (no likelihood
 * accumulation, no explicit alpha/beta); the SPRT form is the H2 upgrade.
 * Counters reset on any phase change.
 *
 *   READY   -> BOOST    axial specific force > liftoff_acc for liftoff_n AND
 *                       nav v_up > liftoff_v; OR (nav invalid) > liftoff_acc
 *                       for liftoff_n_nov. HITL found 50 ms at 3 g alone is
 *                       met by a 6 g / 60 ms handling knock on the pad
 *                       (sensor LPF stretches it); the knock gives only
 *                       ~2 m/s, a motor 8 m/s within 0.04-0.2 s.
 *   BOOST   -> COAST    after t_boost_min, axial f < burnout_acc for burnout_n
 *   COAST   -> DESCENT  v_up < 0 for apogee_n (needs nav valid), OR the
 *                       time-since-burnout backup t_apogee_max
 *   DESCENT -> LANDED   after t_land_min: |f| within land_acc_tol of g, |w|
 *                       below land_gyro, and |v_up| < land_v (GPS-aided only)
 *                       for land_n; OR the backup t_land_max
 *
 * Under a parachute |f| = g too, so the accel test alone cannot detect landing:
 * the velocity term needs a GPS-aided v_up, and the time backup covers no GPS.
 *
 * Pure module: no I/O, no globals.
 * ============================================================================ */
#ifndef DETECT_H_
#define DETECT_H_

#include "phase.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float    liftoff_acc;     /* m/s^2, body axial (+nose)                    */
    uint16_t liftoff_n;
    float    liftoff_v;       /* m/s, nav v_up also required                  */
    uint16_t liftoff_n_nov;   /* persistence when nav is not valid            */
    float    t_boost_min;     /* s                                            */
    float    burnout_acc;     /* m/s^2                                        */
    uint16_t burnout_n;
    uint16_t apogee_n;
    float    t_apogee_max;    /* s after COAST entry                          */
    float    t_land_min;      /* s after DESCENT entry                        */
    float    t_land_max;
    float    land_acc_tol;    /* m/s^2 about g                                */
    float    land_gyro;       /* rad/s                                        */
    float    land_v;          /* m/s                                          */
    uint16_t land_n;
} DetectCfg;

typedef struct {
    float f_axial;            /* body +Z (nose) specific force, m/s^2         */
    float f_norm;             /* |f|, m/s^2                                   */
    float w_norm;             /* |w|, rad/s                                   */
    float v_up;               /* nav vertical velocity, m/s                   */
    bool  v_valid;            /* nav not saturated                            */
    bool  v_gps;              /* v_up GPS-aided (landing test)                */
} DetectIn;

typedef struct {
    DetectCfg c;
    Phase     last;
    uint16_t  n;
    uint32_t  t_ms;           /* time in the current phase (integer: a float
                               * sum of 0.01 drifts ~0.5 s over 300 s)       */
    float     t;              /* t_ms in seconds                              */
} Detect;

void  Detect_Init(Detect *d, const DetectCfg *c);

/* True while the liftoff condition is building in READY (axial acceleration
 * above threshold, not yet confirmed). The arbiter holds READY meanwhile: a
 * fast motor clips the accelerometer and trips accel_rng (SOFT) within 5
 * frames, before the 10-frame liftoff confirmation (HITL, I205W). */
bool  Detect_LiftoffPending(const Detect *d);

/* Returns the requested phase (== now when nothing is detected). */
Phase Detect_Step(Detect *d, Phase now, const DetectIn *in, float dt);

#endif /* DETECT_H_ */
