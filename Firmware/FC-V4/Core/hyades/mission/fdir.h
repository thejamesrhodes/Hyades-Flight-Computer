/* ============================================================================
 * fdir.h  —  Consistency engine + fault arbiter inputs (FDS-01 §2, §4, §6)
 *
 * P2 "one engine, many instances": every monitor is a row in a const table
 * (fdir.c) and is evaluated by the same rule
 *
 *     fail  if  r not in [lo, hi]  for >= n_persist consecutive evaluations
 *
 * where r is a scalar residual computed by the caller. Adding a failure mode
 * adds a table row (+ one Fdir_Eval call), not a subsystem.
 *
 * P3 "detectors report; one arbiter acts": this module only keeps the status
 * bits. The application (app.c) is the arbiter: it maps the summary below to
 * phase transitions and the control-mode ceiling.
 *
 * HARD trips are latched in a value+complement word (FDS-01 §6.2); the latch
 * never clears (reset needed). SOFT failures clear when the residual returns.
 *
 * Pure module: no I/O, no globals (the table is const).
 * ============================================================================ */
#ifndef FDIR_H_
#define FDIR_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum { SEV_INFO = 0, SEV_SOFT, SEV_HARD } FdirSev;

/* What a failing monitor removes from the control system. */
typedef enum { EFF_NONE = 0, EFF_NO_ESTIMATOR, EFF_NO_IMU } FdirEffect;

/* Highest control capability still supported by healthy inputs. */
typedef enum { CAP_NONE = 0, CAP_GYRO_ONLY, CAP_FULL } FdirCap;

typedef enum {
    MON_IMU_READ = 0,   /* r = 1 on a failed IMU read, else 0                 */
    MON_ACCEL_RANGE,    /* r = |a| (m/s^2): below sensor saturation           */
    MON_GYRO_RANGE,     /* r = |w| (dps):   below sensor saturation           */
    MON_LOOP_TIME,      /* r = frame compute time (us) vs 80 % budget (B1)    */
    MON_OVERRUN,        /* r = 1 if the frame started late (B2)               */
    MON_EST_FINITE,     /* r = 1 if any estimator state/covariance non-finite */
    MON_EST_COV,        /* r = tilt error variance (rad^2): converged, sane   */
    MON_GPS_LINK,       /* r = 1 if no valid NMEA within timeout (advisory)   */
    MON_COUNT
} MonId;

typedef struct {
    const char *name;
    float      lo, hi;          /* acceptance band for the residual           */
    uint16_t   n_persist;       /* consecutive out-of-band evaluations        */
    uint8_t    sev_pad;         /* FdirSev on the ground                      */
    uint8_t    sev_flight;      /* FdirSev in BOOST/COAST/DESCENT             */
    uint8_t    effect;          /* FdirEffect while failing                   */
} MonCfg;

typedef struct {
    uint16_t count[MON_COUNT];  /* consecutive out-of-band evaluations        */
    uint16_t trips[MON_COUNT];  /* number of pass->fail transitions (diag)    */
    float    last_r[MON_COUNT]; /* last residual (diag/telemetry)             */
    uint32_t active;            /* bit per monitor: currently failing         */
    uint32_t latch, latch_inv;  /* latched HARD trips, value + complement     */
} Fdir;

void          Fdir_Init(Fdir *f);
const MonCfg *Fdir_Config(MonId id);

/* Evaluate one monitor. Returns true while it is failing. */
bool    Fdir_Eval(Fdir *f, MonId id, float r, bool in_flight);

/* Latched HARD bits. Returns false if the protected word is corrupt. */
bool    Fdir_LatchRead(const Fdir *f, uint32_t *latch);

/* Worst severity among active monitors and the latch (corrupt latch = HARD). */
FdirSev Fdir_Worst(const Fdir *f, bool in_flight);
uint32_t Fdir_HardMask(bool in_flight);    /* monitors that latch in this regime */
/* Operator clear (pad only; caller checks the phase): latch, active bits and
 * persistence counts reset. Trip counters are kept for diagnosis. */
void    Fdir_ClearLatch(Fdir *f);

/* Control capability left after removing what the active monitors affect. */
FdirCap Fdir_Capability(const Fdir *f);

#endif /* FDIR_H_ */
