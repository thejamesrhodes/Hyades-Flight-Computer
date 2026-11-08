/* ============================================================================
 * flight.h  —  Portable per-frame flight pipeline (ADR-004)
 *
 * One call per frame, after the sensor read:
 *   estimate (IEKF + NAV) -> monitors (FDIR) -> event detectors -> arbiter
 *   (phase + control-mode ceiling) -> control law -> fin commands.
 *
 * This is the code that flies. It was lifted out of app.c so that exactly the
 * same object code path runs on the target (app.c calls Flight_Step) and on
 * the host in software-in-the-loop Monte Carlo (Tools/sitl.py loads it as a
 * shared library). No HAL, no I/O, no globals: every side effect leaves
 * through the event callback or the output fields.
 *
 * Ordering rules carried over from app.c (each one was a HITL-found defect):
 *  - detectors run BEFORE the pad demotion (liftoff lost to SOFT demotion);
 *  - landing detection runs despite a HARD fault in DESCENT;
 *  - an operator fault clear is served AFTER this frame's monitors, so
 *    "still failing" means failing now.
 * ============================================================================ */
#ifndef FLIGHT_H_
#define FLIGHT_H_

#include <stdint.h>
#include <stdbool.h>
#include "phase.h"
#include "fdir.h"
#include "iekf.h"
#include "control.h"
#include "nav.h"
#include "detect.h"

/* Event codes (also the EVENT log record code). */
typedef enum { EV_BOOT = 1, EV_PHASE, EV_TRIP, EV_CLEAR, EV_MODE, EV_LOG, EV_CORRUPT, EV_MAINT } EventCode;

typedef void (*FlightEventFn)(void *ctx, EventCode code, uint32_t arg, const char *text);

/* Result of an operator fault clear (served inside Flight_Step). */
typedef enum { FCLR_NONE = 0, FCLR_OK, FCLR_IN_FLIGHT, FCLR_STILL_ACTIVE, FCLR_RECOVER_REFUSED } FlightClear;

typedef struct {
    bool     imu_ok;
    float    accel[3];        /* body specific force, m/s^2                    */
    float    gyro_dps[3];     /* body rate, boot-bias-corrected, dps           */
    uint32_t loop_us;         /* previous frame's compute time                 */
    bool     overran;         /* previous frame overran                        */
    bool     skip_overrun;    /* one deliberate late frame (maintenance)       */
    bool     gps_enabled;
    bool     gps_alive;       /* NMEA seen within GPS_TIMEOUT_MS               */
    bool     gps_new;         /* a new GGA arrived this frame ...              */
    bool     gps_good;        /* ... with usable fix quality                   */
    float    gps_alt_msl;     /* m                                             */
    uint32_t now_ms;
    bool     clear_req;       /* operator 'faults clear'                       */
    bool     ready_inhibit;   /* a pad precondition outside the monitors (flight
                               * log not armable) holds SETTLING: READY needs
                               * the full hold with this false               */
} FlightIn;

typedef struct {
    PhaseWord  ph;
    Fdir       fdir;
    IEKF_State iekf;
    Controller ctrl;
    Nav        nav;
    Detect     det;
    float      fin[4];         /* normalised fin commands APPLIED [-1, 1]      */
    float      fin_cmd[4];     /* law output (= fin unless shadow mode)        */
    bool       shadow;         /* CFG_CTRL_SHADOW: law runs, canards held at 0 */
    float      gyro_rad[3];
    bool       acc_applied;
    bool       bench_ready;    /* CFG_BENCH_ACTUATE_READY                     */
    uint32_t   ready_hold;     /* fault-free frames accumulated in SETTLING    */
    uint32_t   ready_hold_n;   /* frames required                              */
    uint32_t   last_active;
    float      f_last[3];
    uint32_t   f_gap;
    uint32_t   bridge_n;
    uint32_t   degraded_until_ms;
    FlightClear last_clear;
    FlightEventFn ev;
    void      *ev_ctx;
} Flight;

void  Flight_Init(Flight *f, const NavCfg *nav, const DetectCfg *det, uint32_t ready_hold_n,
                  uint32_t bridge_n, bool bench_ready, FlightEventFn ev, void *ev_ctx);
void  Flight_Step(Flight *f, const FlightIn *in, float dt);
Phase Flight_Phase(const Flight *f);              /* PH_LOCKOUT if the word is corrupt */
void  Flight_GoPhase(Flight *f, Phase to);        /* request, logs the edge           */
uint16_t Flight_FaultBits(const Flight *f);       /* active | latched                 */
const char *Flight_ModeName(CtrlMode m);

#endif /* FLIGHT_H_ */
