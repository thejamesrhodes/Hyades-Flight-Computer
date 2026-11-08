/* ============================================================================
 * flight.c  —  see flight.h. Logic moved verbatim from app.c (task_estimate,
 * task_fdir, task_control, faults_clear_serve) so behaviour is unchanged;
 * only the I/O was replaced by FlightIn fields and the event callback.
 * ============================================================================ */
#include "flight.h"
#include "board.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define D2R 0.017453292519943295f

static void emit(Flight *f, EventCode c, uint32_t a, const char *t)
{
    if (f->ev != NULL) { f->ev(f->ev_ctx, c, a, t); }
}

Phase Flight_Phase(const Flight *f)
{
    Phase p;
    return Phase_Read(&f->ph, &p) ? p : PH_LOCKOUT;
}

const char *Flight_ModeName(CtrlMode m)
{
    return (m == CTRL_MODE_FULL) ? "FULL" : ((m == CTRL_MODE_RATE_DAMP) ? "RDAMP" : "OFF");
}

void Flight_GoPhase(Flight *f, Phase to)
{
    const Phase from = Flight_Phase(f);
    if ((from != to) && Phase_Transition(&f->ph, to)) {
        char t[26];
        (void)snprintf(t, sizeof t, "%s->%s", Phase_Name(from), Phase_Name(to));
        emit(f, EV_PHASE, (uint32_t)to, t);
    }
}

uint16_t Flight_FaultBits(const Flight *f)
{
    uint32_t l = 0xFFFFu;
    (void)Fdir_LatchRead(&f->fdir, &l);
    return (uint16_t)((f->fdir.active | l) & 0xFFFFu);
}

void Flight_Init(Flight *f, const NavCfg *nav, const DetectCfg *det, uint32_t ready_hold_n,
                 uint32_t bridge_n, bool bench_ready, FlightEventFn ev, void *ev_ctx)
{
    memset(f, 0, sizeof *f);
    Phase_Init(&f->ph);
    Fdir_Init(&f->fdir);
    Ctrl_Init(&f->ctrl);
    IEKF_Init(&f->iekf);
    Nav_Init(&f->nav, nav);
    Detect_Init(&f->det, det);
    f->ready_hold_n = ready_hold_n;
    f->bridge_n = bridge_n;
    f->bench_ready = bench_ready;
    f->shadow = (CFG_CTRL_SHADOW != 0);
    f->ev = ev;
    f->ev_ctx = ev_ctx;
}

/* GPS -> nav: site elevation on the pad, altitude aiding after apogee only. */
static void nav_gps(Flight *f, const FlightIn *in, Phase ph)
{
    if (!in->gps_enabled || !in->gps_new || !in->gps_good) { return; }
    if ((ph == PH_SETTLING) || (ph == PH_READY)) {
        Nav_PadGps(&f->nav, in->gps_alt_msl);
    } else if ((ph == PH_DESCENT) || (ph == PH_LANDED)) {
        (void)Nav_GpsAlt(&f->nav, in->gps_alt_msl);
    } else {
        /* boost/coast: not used (lock loss, 1 Hz) */
    }
}

static void estimate(Flight *f, const FlightIn *in, float dt)
{
    f->acc_applied = false;
    if (in->imu_ok) {
        IEKF_Predict(&f->iekf, in->gyro_dps, dt);
        f->acc_applied = IEKF_UpdateAccel(&f->iekf, in->accel);
        const Phase ph = Flight_Phase(f);
        const bool pad = !Phase_InFlight(ph) && (ph != PH_CATO_SAFE);
        Nav_Step(&f->nav, f->iekf.q, in->accel, f->iekf.omega, dt, pad);
        nav_gps(f, in, ph);
        for (uint32_t k = 0u; k < 3u; k++) { f->f_last[k] = in->accel[k]; }
        f->f_gap = 0u;
    } else if ((f->f_gap < f->bridge_n) && Phase_InFlight(Flight_Phase(f))) {
        /* Short read gap in flight: integrate the last good specific force
         * (HITL: a 50 ms boost dropout otherwise cost ~13 % in q_hat). */
        f->f_gap++;
        Nav_Step(&f->nav, f->iekf.q, f->f_last, f->iekf.omega, dt, false);
    } else {
        f->f_gap++;
    }
}

static void serve_clear(Flight *f, Phase ph)
{
    if (Phase_InFlight(ph) || (ph == PH_CATO_SAFE)) { f->last_clear = FCLR_IN_FLIGHT; return; }
    if ((f->fdir.active & Fdir_HardMask(false)) != 0u) { f->last_clear = FCLR_STILL_ACTIVE; return; }
    Fdir_ClearLatch(&f->fdir);
    f->last_active = f->fdir.active;
    if (ph == PH_LOCKOUT) {
        if (!Phase_Recover(&f->ph)) { f->last_clear = FCLR_RECOVER_REFUSED; return; }
        f->ready_hold = 0u;
    }
    emit(f, EV_CLEAR, 0u, "faults cleared (operator)");
    f->last_clear = FCLR_OK;
}

static void monitor_and_arbitrate(Flight *f, const FlightIn *in, float dt)
{
    Phase ph;
    if (!Phase_Read(&f->ph, &ph)) {                /* protected word corrupt */
        /* STM-01 F1: decide flight vs pad from the detector's last good phase,
         * so a corrupt word in flight lands in CATO_SAFE (which can still
         * reach LANDED), not LOCKOUT (terminal). */
        Phase_ForceSafe(&f->ph, Phase_InFlight(f->det.last));
        emit(f, EV_CORRUPT, 0u, "phase word corrupt");
        ph = Flight_Phase(f);
    }
    const bool fl = Phase_InFlight(ph);

    const float an = sqrtf((in->accel[0] * in->accel[0]) + (in->accel[1] * in->accel[1])
                           + (in->accel[2] * in->accel[2]));
    const float wn = sqrtf((in->gyro_dps[0] * in->gyro_dps[0]) + (in->gyro_dps[1] * in->gyro_dps[1])
                           + (in->gyro_dps[2] * in->gyro_dps[2]));
    (void)Fdir_Eval(&f->fdir, MON_IMU_READ,    in->imu_ok ? 0.0f : 1.0f, fl);
    (void)Fdir_Eval(&f->fdir, MON_ACCEL_RANGE, an, fl);
    (void)Fdir_Eval(&f->fdir, MON_GYRO_RANGE,  wn, fl);
    (void)Fdir_Eval(&f->fdir, MON_LOOP_TIME,   (float)in->loop_us, fl);
    (void)Fdir_Eval(&f->fdir, MON_OVERRUN,     (in->overran && !in->skip_overrun) ? 1.0f : 0.0f, fl);
    (void)Fdir_Eval(&f->fdir, MON_EST_FINITE,  IEKF_IsFinite(&f->iekf) ? 0.0f : 1.0f, fl);
    (void)Fdir_Eval(&f->fdir, MON_EST_COV,     IEKF_TiltVar(&f->iekf), fl);
    if (in->gps_enabled) {
        (void)Fdir_Eval(&f->fdir, MON_GPS_LINK, in->gps_alive ? 0.0f : 1.0f, fl);
    }

    const uint32_t changed = f->fdir.active ^ f->last_active;
    for (uint32_t i = 0u; i < (uint32_t)MON_COUNT; i++) {
        if ((changed & (1u << i)) == 0u) { continue; }
        char t[26];
        const bool on = (f->fdir.active & (1u << i)) != 0u;
        (void)snprintf(t, sizeof t, "%s %s", on ? "trip" : "clear", Fdir_Config((MonId)i)->name);
        emit(f, on ? EV_TRIP : EV_CLEAR, i, t);
    }
    f->last_active = f->fdir.active;
    f->last_clear = FCLR_NONE;
    if (in->clear_req) { serve_clear(f, ph); ph = Flight_Phase(f); }

    /* Detectors before the pad demotion; landing allowed despite HARD. */
#if CFG_PHASE_DETECT
    if ((Fdir_Worst(&f->fdir, fl) != SEV_HARD) || (ph == PH_DESCENT) || (ph == PH_CATO_SAFE)) {
        DetectIn d;
        memset(&d, 0, sizeof d);
        d.imu_ok = in->imu_ok;
        if (in->imu_ok) {
            d.f_axial = in->accel[2];              /* body +Z = nose */
            d.f_norm = an;
            d.w_norm = wn * D2R;
        }
        d.v_up = Nav_VUp(&f->nav);
        d.v_valid = Nav_QValid(&f->nav);
        d.v_gps = (f->nav.flags & NAV_F_GPS_AIDED) != 0u;
        const Phase to = Detect_Step(&f->det, ph, &d, dt);
        if (to != ph) { Flight_GoPhase(f, to); ph = Flight_Phase(f); }
    }
#endif

    const FdirSev worst = Fdir_Worst(&f->fdir, Phase_InFlight(ph));
    if (worst == SEV_HARD) {
        Flight_GoPhase(f, fl ? PH_CATO_SAFE : PH_LOCKOUT);
    } else if (ph == PH_SETTLING) {
        f->ready_hold = ((worst == SEV_INFO) && !in->ready_inhibit) ? (f->ready_hold + 1u) : 0u;
        if (f->ready_hold >= f->ready_hold_n) { Flight_GoPhase(f, PH_READY); }
    } else if ((ph == PH_READY) && (worst == SEV_SOFT) && !Detect_LiftoffPending(&f->det)) {
        f->ready_hold = 0u;
        Flight_GoPhase(f, PH_SETTLING);           /* demote; re-promotion needs the full hold */
    } else {
        /* no transition */
    }

    /* Control-mode ceiling (degraded-mode ladder). */
    CtrlMode m = CTRL_MODE_OFF;
    if (Phase_ActuationAllowed(Flight_Phase(f), f->bench_ready)) {
        const FdirCap cap = Fdir_Capability(&f->fdir);
        m = (cap == CAP_FULL) ? CTRL_MODE_FULL : ((cap == CAP_GYRO_ONLY) ? CTRL_MODE_RATE_DAMP : CTRL_MODE_OFF);
    }
    if (m != f->ctrl.mode) {
        char t[26];
        (void)snprintf(t, sizeof t, "ctrl %s->%s", Flight_ModeName(f->ctrl.mode), Flight_ModeName(m));
        Ctrl_SetMode(&f->ctrl, m);
        emit(f, EV_MODE, (uint32_t)m, t);
    }
    if ((worst >= SEV_SOFT) || ((m != CTRL_MODE_FULL) && Phase_ActuationAllowed(Flight_Phase(f), f->bench_ready))) {
        f->degraded_until_ms = in->now_ms + 1000u;
    }
}

void Flight_Step(Flight *f, const FlightIn *in, float dt)
{
    for (uint32_t k = 0u; k < 3u; k++) { f->gyro_rad[k] = in->gyro_dps[k] * D2R; }
    estimate(f, in, dt);
    monitor_and_arbitrate(f, in, dt);
    Ctrl_SetQbar(&f->ctrl, f->nav.qbar, Nav_QValid(&f->nav) && (f->ctrl.mode == CTRL_MODE_FULL));
    Ctrl_Update(&f->ctrl, f->iekf.q, f->iekf.omega, f->gyro_rad, dt, f->fin_cmd);
    for (uint32_t k = 0u; k < 4u; k++) { f->fin[k] = f->shadow ? 0.0f : f->fin_cmd[k]; }
}
