/* ============================================================================
 * sitl.c  —  Software-in-the-loop 6DOF Monte Carlo harness (ADR-004 §6)
 *
 * The flight code is the real thing: Core/hyades (flight.c and everything it
 * calls) compiled for the host, driven frame by frame exactly as app.c drives
 * it on the target. Around it, a 6DOF plant written for this vehicle:
 *
 *   - forward body (rigid, 6DOF) + free-spinning rear fin can (1 roll DOF),
 *     coupled by bearing friction and the can's gyroscopic moment;
 *   - aero from OpenRocket (Cd(M), CP(M), total CN_alpha) split by Barrowman
 *     into nose + rear fins (on the body, with pitch damping) and the four
 *     canards, which are modelled per fin with their local flow (so canard
 *     roll damping, roll/pitch/yaw cross-coupling and stall come out of the
 *     geometry rather than being added);
 *   - rear-fin misalignment: a net cant spins the can; a single misaligned
 *     fin adds a side force that rotates with the can (roll lock-in path);
 *   - thrust misalignment, CG offset, wind (shear + turbulence + discrete gust),
 *     launch rail, parachute on a shock cord (swing), ground impact;
 *   - QMI8658A model: bias/scale/misalignment/g-sensitivity, measured noise,
 *     vibration added at the internal 7.17 kHz rate, clipping at full scale,
 *     2nd-order LPF, decimation to ODR, 16-bit quantisation, lever arm;
 *   - four servos: canard_servo_reduced port (delay, 2nd order, dead band,
 *     rate limit, lash, quantisation, travel), frame-synchronous pulses,
 *     supply-voltage-dependent slew, and injectable faults.
 *
 * Usage:  sitl key=value ...     (see the defaults in cfg_defaults())
 * Output: one line of key=value metrics on stdout; trace=FILE writes a CSV.
 * Units SI unless the name says otherwise. Body: z out of the nose, x pitch,
 * y yaw (board.h §1). Stations x_* are measured AFT from the nose tip.
 * ============================================================================ */
#include "flight.h"
#include "board.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define G0   9.80665
#define D2R  0.017453292519943295
#define R2D  57.29577951308232
#define MAXT 600
#define MAXA 64

/* ------------------------------------------------------------------ rng -- */
static unsigned long long rs[2];
static double urand(void)
{
    unsigned long long s1 = rs[0]; const unsigned long long s0 = rs[1];
    rs[0] = s0; s1 ^= s1 << 23; rs[1] = s1 ^ s0 ^ (s1 >> 17) ^ (s0 >> 26);
    return (double)((rs[1] + s0) >> 11) * (1.0 / 9007199254740992.0);
}
static double nrand(void)
{
    double u = urand(); if (u < 1e-300) { u = 1e-300; }
    return sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * urand());
}
static void seed_rng(unsigned long long s)
{
    unsigned long long z = s + 0x9E3779B97F4A7C15ull;
    for (int i = 0; i < 2; i++) {
        z += 0x9E3779B97F4A7C15ull; unsigned long long x = z;
        x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull; x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
        rs[i] = x ^ (x >> 31);
    }
}

/* --------------------------------------------------------------- config -- */
typedef struct {
    /* run */
    unsigned long long seed; double pad_hold, t_max, dt_asc, dt_desc;
    char motor[256], aero[256], trace[256];
    /* vehicle (ADR-004 assumption register) */
    double m_dry, x_cg_dry, x_mot, It_dry, Ixx_body, I_can, L_total, d;
    double x_canard, x_fin, x_nose_cp, x_attach, x_imu, r_imu;
    double CNa_nose, CNa_fins, kf_can, ycp_can, Cld_f, Clp_f, kff_fin, Cf, L_body;
    double Cld_scale, Clp_scale, CNa_scale, cp_shift, cg_shift, Cd_scale;
    double ixx_scale, it_scale, mass_scale, thrust_scale, alpha_stall;
    /* asymmetries */
    double cl0_body, can_cant_deg, fin1_mis_deg, thrust_mis_deg, thrust_az_deg, cg_off_y;
    double bear_coul, bear_visc;
    /* environment */
    double site_elev, wind, wind_dir, turb, gust_t, gust_amp, gust_dur, rail_len, rail_tilt, rail_az;
    /* recovery */
    double chute_delay, chute_cd_a, cord_len, ground_k, ground_c, canopy_swing, collapse_t;
    /* imu */
    double arw, vrw, acc_bias, gyro_bias, sf, misal_deg, gsens;
    double vib_boost_g, vib_res_g, vib_res_hz, vib_gyro_dps, vib_coast_g, vib_chute_g;
    /* servo (sim_params.json Servo, BMS-127WV+) */
    double sv_rate, sv_wn, sv_zeta, sv_tau, sv_db, sv_lash, sv_res, sv_travel, sv_volt_k, sv_extra;
    int    fault_fin; double fault_t; char fault_kind[16]; double fault_val;
    /* gps */
    int    gps_on; double gps_sd;
    /* firmware switches */
    int    shadow; int fin_deg;
} Cfg;
static Cfg C;

static void cfg_defaults(void)
{
    memset(&C, 0, sizeof C);
    C.seed = 1; C.pad_hold = 6.0; C.t_max = 400.0; C.dt_asc = 2.5e-4; C.dt_desc = 1e-3;
    strcpy(C.motor, "motors/G79W.csv"); strcpy(C.aero, "aero.csv");
    /* mass properties (OpenRocket "62mm Canard Control V1.0.0", G79W loaded:
     * 897.6 g, CG 46.07 cm, transverse I 0.0734). Dry = less the motor.
     * LD-01 §2: solved from the OpenRocket G79W export at ignition and burnout
     * (loaded 897.6 g / CG 0.4607 / It 0.0734; burnout CG 0.4339). Earlier
     * values 0.4278 / 0.8273 / 0.065 put the loaded CG 2.2 cm aft. */
    C.m_dry = 0.7736; C.x_cg_dry = 0.4007; C.x_mot = 0.835; C.It_dry = 0.0530;
    C.Ixx_body = 2.2e-4; C.I_can = 2.6e-4; C.L_total = 0.886; C.d = 0.062;
    C.x_canard = 0.192; C.x_fin = 0.812; C.x_nose_cp = 0.0466; C.x_attach = 0.18;
    C.x_imu = 0.22; C.r_imu = 0.005;
    C.CNa_nose = 2.0; C.CNa_fins = 8.47;
    C.kf_can = 7.88e-4; C.ycp_can = 0.0390;        /* per-canard lift (m^2/rad), spanwise CP */
    C.Cld_f = 8.747; C.Clp_f = -17.276;            /* rear fins on the can (strip theory)   */
    C.kff_fin = 0.0128;                            /* single rear fin side force m^2/rad    */
    C.Cf = 0.003; C.L_body = 0.8;
    C.Cld_scale = 1; C.Clp_scale = 1; C.CNa_scale = 1; C.Cd_scale = 1;
    C.ixx_scale = 1; C.it_scale = 1; C.mass_scale = 1; C.thrust_scale = 1; C.alpha_stall = 15.0;
    C.bear_coul = 0.0075; C.bear_visc = 2e-5;
    C.site_elev = 110; C.wind = 3; C.wind_dir = 270; C.turb = 0.15; C.rail_len = 1.5;
    C.rail_tilt = 3; C.rail_az = 270; C.gust_t = -1;
    C.chute_delay = 1.0; C.canopy_swing = 8.0; C.collapse_t = 5.0; C.chute_cd_a = 0.7 * M_PI * 0.09; C.cord_len = 1.5;
    C.ground_k = 1500.0; C.ground_c = 30.0;     /* soft field: ~30 g peak at 6 m/s */
    C.arw = 1.49e-4; C.vrw = 7.0e-4;              /* rad/s/rtHz, m/s^2/rtHz (measured)     */
    C.acc_bias = 0.03; C.gyro_bias = 0.05 * D2R; C.sf = 0.005; C.misal_deg = 0.3; C.gsens = 0.03 * D2R / G0;
    C.vib_boost_g = 1.5; C.vib_res_g = 1.0; C.vib_res_hz = 450; C.vib_gyro_dps = 3; C.vib_coast_g = 0.3; C.vib_chute_g = 0.05;
    C.sv_rate = 23.65; C.sv_wn = 80; C.sv_zeta = 0.7; C.sv_tau = 0.014; C.sv_db = 0.00196; C.sv_lash = 0.002;
    C.sv_res = 5.1e-4; C.sv_travel = 0.2094; C.sv_volt_k = 1.0; C.sv_extra = 1.9e-3;
    C.fault_fin = -1; C.fault_t = 1e9; strcpy(C.fault_kind, "none");
    C.gps_on = 1; C.gps_sd = 4.0;
    C.fin_deg = (int)FIN_DEFLECT_DEG;
}

static void cfg_set(const char *k, const char *v)
{
#define D(n) if (!strcmp(k, #n)) { C.n = atof(v); return; }
#define I(n) if (!strcmp(k, #n)) { C.n = atoi(v); return; }
#define S(n) if (!strcmp(k, #n)) { snprintf(C.n, sizeof C.n, "%s", v); return; }
    if (!strcmp(k, "seed")) { C.seed = strtoull(v, NULL, 10); return; }
    S(motor) S(aero) S(trace) S(fault_kind)
    D(pad_hold) D(t_max) D(dt_asc) D(dt_desc) D(m_dry) D(x_cg_dry) D(x_mot) D(It_dry) D(Ixx_body) D(I_can)
    D(L_total) D(x_canard) D(x_fin) D(x_attach) D(x_imu) D(r_imu) D(CNa_nose) D(CNa_fins) D(kf_can)
    D(Cld_f) D(Clp_f) D(kff_fin) D(Cld_scale) D(Clp_scale) D(CNa_scale) D(cp_shift) D(cg_shift) D(Cd_scale)
    D(ixx_scale) D(it_scale) D(mass_scale) D(thrust_scale) D(alpha_stall) D(cl0_body) D(can_cant_deg)
    D(fin1_mis_deg) D(thrust_mis_deg) D(thrust_az_deg) D(cg_off_y) D(bear_coul) D(bear_visc) D(site_elev)
    D(wind) D(wind_dir) D(turb) D(gust_t) D(gust_amp) D(gust_dur) D(rail_len) D(rail_tilt) D(rail_az)
    D(ground_k) D(ground_c) D(collapse_t) D(chute_delay) D(canopy_swing) D(chute_cd_a) D(cord_len) D(arw) D(vrw) D(acc_bias) D(gyro_bias) D(sf) D(misal_deg) D(gsens)
    D(vib_boost_g) D(vib_res_g) D(vib_res_hz) D(vib_gyro_dps) D(vib_coast_g) D(vib_chute_g)
    D(sv_rate) D(sv_wn) D(sv_zeta) D(sv_tau) D(sv_db) D(sv_lash) D(sv_res) D(sv_travel) D(sv_volt_k) D(sv_extra)
    I(fault_fin) D(fault_t) D(fault_val) I(gps_on) D(gps_sd) I(shadow) I(fin_deg)
    fprintf(stderr, "unknown key %s\n", k); exit(2);
#undef D
#undef I
#undef S
}

/* --------------------------------------------------------------- tables -- */
typedef struct { int n; double t[MAXT], F[MAXT], m[MAXT]; double prop, case_m, tb; } MotorT;
typedef struct { int n; double M[MAXA], Cd[MAXA], xcp[MAXA], CNa[MAXA]; } AeroT;
static MotorT MO; static AeroT AE;

static double interp(const double *x, const double *y, int n, double v)
{
    if (v <= x[0]) { return y[0]; }
    if (v >= x[n - 1]) { return y[n - 1]; }
    int i = 1; while ((i < n - 1) && (x[i] < v)) { i++; }
    const double f = (v - x[i - 1]) / (x[i] - x[i - 1]);
    return y[i - 1] + (f * (y[i] - y[i - 1]));
}

/* motor table: "t thrust mass" rows (mass = total motor mass, kg) */
static void load_motor(const char *p)
{
    FILE *f = fopen(p, "r"); if (!f) { fprintf(stderr, "motor %s\n", p); exit(2); }
    char ln[256]; MO.n = 0;
    while (fgets(ln, sizeof ln, f) && (MO.n < MAXT)) {
        if ((ln[0] == '#') || (ln[0] == '\n')) { continue; }
        if (sscanf(ln, "%lf %lf %lf", &MO.t[MO.n], &MO.F[MO.n], &MO.m[MO.n]) == 3) { MO.n++; }
    }
    fclose(f);
    MO.tb = MO.t[MO.n - 1]; MO.case_m = MO.m[MO.n - 1]; MO.prop = MO.m[0] - MO.case_m;
}
/* aero table: "Mach Cd xcp_m CNa" rows (OpenRocket, total vehicle at alpha ~0) */
static void load_aero(const char *p)
{
    FILE *f = fopen(p, "r"); if (!f) { fprintf(stderr, "aero %s\n", p); exit(2); }
    char ln[256]; AE.n = 0;
    while (fgets(ln, sizeof ln, f) && (AE.n < MAXA)) {
        if ((ln[0] == '#') || (ln[0] == '\n')) { continue; }
        if (sscanf(ln, "%lf %lf %lf %lf", &AE.M[AE.n], &AE.Cd[AE.n], &AE.xcp[AE.n], &AE.CNa[AE.n]) == 4) { AE.n++; }
    }
    fclose(f);
}

/* ------------------------------------------------------------ vec utils -- */
typedef struct { double x, y, z; } V3;
static V3 v3(double x, double y, double z) { V3 r = { x, y, z }; return r; }
static V3 add(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static V3 sub(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static V3 scl(V3 a, double s) { return v3(a.x * s, a.y * s, a.z * s); }
static double dot(V3 a, V3 b) { return (a.x * b.x) + (a.y * b.y) + (a.z * b.z); }
static V3 crs(V3 a, V3 b) { return v3((a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x)); }
static double nrm(V3 a) { return sqrt(dot(a, a)); }
typedef struct { double w, x, y, z; } Q4;
static V3 qrot(Q4 q, V3 v)   /* body -> world */
{
    const V3 u = v3(q.x, q.y, q.z);
    const V3 t = scl(crs(u, v), 2.0);
    return add(add(v, scl(t, q.w)), crs(u, t));
}
static V3 qrot_t(Q4 q, V3 v) { Q4 c = { q.w, -q.x, -q.y, -q.z }; return qrot(c, v); }
static Q4 qint(Q4 q, V3 w, double dt)   /* q <- q (x) exp(w dt / 2) */
{
    const double a = nrm(w) * dt * 0.5; double s = 0.5 * dt, c = 1.0;
    if (a > 1e-9) { s = sin(a) / nrm(w); c = cos(a); }
    const Q4 e = { c, w.x * s, w.y * s, w.z * s };
    Q4 r = { (q.w * e.w) - (q.x * e.x) - (q.y * e.y) - (q.z * e.z), (q.w * e.x) + (q.x * e.w) + (q.y * e.z) - (q.z * e.y),
             (q.w * e.y) - (q.x * e.z) + (q.y * e.w) + (q.z * e.x), (q.w * e.z) + (q.x * e.y) - (q.y * e.x) + (q.z * e.w) };
    const double n = sqrt((r.w * r.w) + (r.x * r.x) + (r.y * r.y) + (r.z * r.z));
    r.w /= n; r.x /= n; r.y /= n; r.z /= n; return r;
}

/* ---------------------------------------------------------------- servo -- */
typedef struct {
    double th, thd, lash, cmd_now, buf[256]; int nd, w; double t_next_cmd, cmd_pending;
    int fault; /* 0 none 1 stuck 2 hardover 3 slow 4 float 5 offset */
} Servo;
static void servo_init(Servo *s, double dt)
{
    memset(s, 0, sizeof *s);
    s->nd = (int)lround(C.sv_tau / dt); if (s->nd < 1) { s->nd = 1; } if (s->nd > 255) { s->nd = 255; }
}
static double servo_step(Servo *s, double cmd, double dt)
{
    if (s->fault == 1) { return s->lash; }                       /* stuck at the current angle */
    if (s->fault == 2) { cmd = C.sv_travel * ((C.fault_val >= 0) ? 1.0 : -1.0); }
    if (s->fault == 5) { cmd += C.fault_val * D2R; }
    const double u = round(cmd / C.sv_res) * C.sv_res;
    s->buf[s->w] = u; const double cd = s->buf[(s->w - s->nd + 256) % 256]; s->w = (s->w + 1) % 256;
    const double e = cd - s->th;
    const double edz = (fabs(e) <= C.sv_db) ? 0.0 : (e - copysign(C.sv_db, e));
    const double rate = C.sv_rate * C.sv_volt_k * ((s->fault == 3) ? 0.25 : 1.0);
    const double thdd = (C.sv_wn * C.sv_wn * edz) - (2.0 * C.sv_zeta * C.sv_wn * s->thd);
    s->thd += thdd * dt; if (s->thd > rate) { s->thd = rate; } if (s->thd < -rate) { s->thd = -rate; }
    s->th += s->thd * dt;
    if (s->th > C.sv_travel) { s->th = C.sv_travel; s->thd = 0; }
    if (s->th < -C.sv_travel) { s->th = -C.sv_travel; s->thd = 0; }
    const double b = 0.5 * C.sv_lash;
    if ((s->th - s->lash) > b) { s->lash = s->th - b; } else if ((s->lash - s->th) > b) { s->lash = s->th + b; }
    return s->lash;
}

/* ------------------------------------------------------------------ IMU -- */
#define IMU_DECIM 16
typedef struct {
    double odr, fs_int, t_next, a_lp[3][2], g_lp[3][2];   /* 2nd-order LPF states (biquad DF2) */
    double b0, b1, b2, a1, a2;
    double ba[3], bg[3], sfa[3], sfg[3], M[3][3];
    double res_x[3][2], res_b0, res_a1, res_a2, res_gain; /* resonance (2-pole) state */
    float out_a[3], out_g[3]; int k;
    double vib_g, vib_res_g, vib_gyro;
} Imu;
static Imu IM;
static void biquad_lp(double fc, double fs, double *b0, double *b1, double *b2, double *a1, double *a2)
{
    const double K = tan(M_PI * fc / fs), Q = 0.70710678, n = 1.0 / (1.0 + (K / Q) + (K * K));
    *b0 = K * K * n; *b1 = 2.0 * *b0; *b2 = *b0; *a1 = 2.0 * ((K * K) - 1.0) * n; *a2 = (1.0 - (K / Q) + (K * K)) * n;
}
static double bq(double x, double *z, int dummy)
{
    (void)dummy;
    const double y = (IM.b0 * x) + z[0];
    z[0] = (IM.b1 * x) - (IM.a1 * y) + z[1];
    z[1] = (IM.b2 * x) - (IM.a2 * y);
    return y;
}
static void imu_init(void)
{
    memset(&IM, 0, sizeof IM);
    IM.odr = 448.4; IM.fs_int = IM.odr * IMU_DECIM;
    biquad_lp(0.1337 * IM.odr, IM.fs_int, &IM.b0, &IM.b1, &IM.b2, &IM.a1, &IM.a2);
    for (int i = 0; i < 3; i++) {
        IM.ba[i] = C.acc_bias * nrand(); IM.bg[i] = C.gyro_bias * nrand();
        IM.sfa[i] = 1.0 + (C.sf * nrand()); IM.sfg[i] = 1.0 + (C.sf * nrand());
    }
    /* small-angle misalignment matrix */
    const double m = C.misal_deg * D2R;
    const double ex = m * nrand(), ey = m * nrand(), ez = m * nrand();
    double M[3][3] = { { 1, -ez, ey }, { ez, 1, -ex }, { -ey, ex, 1 } };
    memcpy(IM.M, M, sizeof M);
    /* resonance: 2nd-order band-pass, Q = 5, at the internal rate */
    const double w0 = 2.0 * M_PI * C.vib_res_hz / IM.fs_int, al = sin(w0) / (2.0 * 5.0), a0 = 1.0 + al;
    IM.res_b0 = al / a0; IM.res_a1 = -2.0 * cos(w0) / a0; IM.res_a2 = (1.0 - al) / a0;
    /* normalise the resonance generator to unit RMS (measured, 2e4 samples) */
    double z0 = 0, z1 = 0, ss = 0;
    for (int n = 0; n < 20000; n++) {
        const double x = nrand(), r = (IM.res_b0 * x) - (IM.res_a1 * z0) - (IM.res_a2 * z1);
        z1 = z0; z0 = r; ss += r * r;
    }
    IM.res_gain = 1.0 / sqrt(ss / 20000.0);
}
static double clampd(double x, double l) { return (x > l) ? l : ((x < -l) ? -l : x); }
/* advance the sensor across one plant step [t, t+dt) with truth f (m/s^2), w (rad/s) */
static void imu_advance(double t, double dt, V3 f, V3 w)
{
    const double fa[3] = { f.x, f.y, f.z }, wa[3] = { w.x, w.y, w.z };
    const double ti = 1.0 / IM.fs_int;
    const double sa = C.vrw * sqrt(IM.fs_int), sg = C.arw * sqrt(IM.fs_int);
    while (IM.t_next < t + dt) {
        IM.t_next += ti;
        for (int i = 0; i < 3; i++) {
            double a = 0, g = 0;
            for (int j = 0; j < 3; j++) { a += IM.M[i][j] * fa[j]; g += IM.M[i][j] * wa[j]; }
            a = (a * IM.sfa[i]) + IM.ba[i] + (sa * nrand());
            g = (g * IM.sfg[i]) + IM.bg[i] + (C.gsens * fa[2] * ((i == 2) ? 0.3 : 1.0)) + (sg * nrand());
            if (IM.vib_g > 0) {
                const double x = nrand();
                double *z = IM.res_x[i];
                const double r = (IM.res_b0 * x) - (IM.res_a1 * z[0]) - (IM.res_a2 * z[1]);   /* 2-pole resonator */
                z[1] = z[0]; z[0] = r;
                a += G0 * ((IM.vib_g * nrand()) + (IM.vib_res_g * r * IM.res_gain));
                g += IM.vib_gyro * D2R * nrand();
            }
            a = clampd(a, 16.0 * G0); g = clampd(g, 2048.0 * D2R);     /* clips BEFORE the filter */
            const double af = bq(a, IM.a_lp[i], 0), gf = bq(g, IM.g_lp[i], 0);
            if ((IM.k % IMU_DECIM) == 0) {
                const double lsb_a = 16.0 * G0 / 32768.0, lsb_g = 2048.0 / 32768.0;
                IM.out_a[i] = (float)(round(af / lsb_a) * lsb_a);
                IM.out_g[i] = (float)(round((gf * R2D) / lsb_g) * lsb_g);   /* dps */
            }
        }
        IM.k++;
    }
}

/* ---------------------------------------------------------------- plant -- */
typedef struct {
    double t; int ignited, on_rail, chute, landed, rested; double t_ign, t_chute, t_ground, t_rest;
    V3 p, v; Q4 q; V3 w; double pc, phic;   /* can roll rate (inertial about body z), angle */
    double m, xcg, It, Ixx; V3 f_spec, wdot; double qbar, mach, V, alpha;
    double delta[4]; Servo sv[4]; double cmd[4];
    V3 wind_turb; double chute_open, swing_x, swing_y;
    double apogee, t_apogee, v_burn; double imp_g;
    double dep_V, dep_gam, dep_h, dep_aoa, dep_w, v_desc;
} Plant;
static Plant P;

static void isa(double h, double *rho, double *a)
{
    const double T = 288.15 - (0.0065 * fmin(fmax(h, -500.0), 11000.0));
    const double p = 101325.0 * pow(T / 288.15, 5.255877);
    *rho = p / (287.053 * T); *a = sqrt(1.4 * 287.053 * T);
}

static V3 wind_at(double h, double dt)
{
    const double hh = fmax(h, 1.0);
    const double U = C.wind * pow(hh / 10.0, 0.14);
    const double dir = C.wind_dir * D2R;          /* "from" direction, met convention */
    V3 w = v3(-U * sin(dir), -U * cos(dir), 0.0);
    /* turbulence: first-order Gauss-Markov per axis, L/V ~ 1 s, sigma = turb * U */
    const double tau = 1.0, s = C.turb * fmax(U, 1.0), a = exp(-dt / tau);
    P.wind_turb.x = (a * P.wind_turb.x) + (s * sqrt(1.0 - (a * a)) * nrand());
    P.wind_turb.y = (a * P.wind_turb.y) + (s * sqrt(1.0 - (a * a)) * nrand());
    P.wind_turb.z = (a * P.wind_turb.z) + (0.5 * s * sqrt(1.0 - (a * a)) * nrand());
    w = add(w, P.wind_turb);
    if ((C.gust_t >= 0) && P.ignited) {
        const double tg = P.t - C.gust_t;
        if ((tg >= 0) && (tg <= C.gust_dur)) {
            const double u = 0.5 * C.gust_amp * (1.0 - cos(2.0 * M_PI * tg / C.gust_dur));
            w = add(w, v3(u * cos(dir), -u * sin(dir), 0.0));     /* crosswind to the mean wind */
        }
    }
    return w;
}

static void mass_props(void)
{
    double mm = MO.m[0];
    if (P.ignited) { mm = interp(MO.t, MO.m, MO.n, P.t - P.t_ign); }
    const double md = C.m_dry * C.mass_scale;
    P.m = md + mm;
    P.xcg = ((md * (C.x_cg_dry + C.cg_shift)) + (mm * C.x_mot)) / P.m;
    const double It = (C.It_dry * C.it_scale) + (md * pow(C.x_cg_dry + C.cg_shift - P.xcg, 2))
                    + (mm * pow(C.x_mot - P.xcg, 2));
    P.It = It; P.Ixx = C.Ixx_body * C.ixx_scale;
}

static double thrust(void)
{
    if (!P.ignited) { return 0.0; }
    const double tt = P.t - P.t_ign;
    if (tt > MO.tb) { return 0.0; }
    return C.thrust_scale * interp(MO.t, MO.F, MO.n, tt);
}

/* body position of a station relative to the CG */
static V3 st(double x_station, double r_x, double r_y) { return v3(r_x, r_y, P.xcg - x_station); }


/* ---------------------------------------------------------------- loads -- */
/* LD-01: internal loads at body stations by the cut method. The forward part
 * of the vehicle at a cut carries its external non-gravity forces F_i and its
 * inertial reactions -m_j f_j, where f_j is the specific force at node j
 * (f_cg + wdot x r + w x (w x r), gravity cancels). Their resultant is what
 * the aft part must react: axial (compression +), shear, bending moment.
 * The dry mass is lumped from the OpenRocket component tree and calibrated
 * to OpenRocket's m, CG and It; the motor is 4 nodes over its length. */
#define NLD 6
static const double LD_X[NLD] = { 0.100, 0.180, 0.260, 0.320, 0.510, 0.760 };
static const char *const LD_NAME[NLD] = { "nose_joint", "avbay_canard", "canard_body", "coupler_aft", "body_mid", "body_fincan" };
#define NNODE 40
static int n_node; static double nd_x[NNODE], nd_m0[NNODE];
#define NFRC 32
static int n_frc; static double fr_x[NFRC]; static V3 fr_F[NFRC];
typedef struct { double Mb, P, Pt, S, Neq; } LdMax;
static LdMax LDM[NLD]; static double ld_resid_max, ld_lump_It;
static void frc(double x, V3 Fb) { if (n_frc < NFRC) { fr_x[n_frc] = x; fr_F[n_frc] = Fb; n_frc++; } }
static void node(double x, double m) { if (n_node < NNODE) { nd_x[n_node] = x; nd_m0[n_node] = m; n_node++; } }
static void tube(double x0, double x1, double m) { for (int i = 0; i < 4; i++) { node(x0 + ((x1 - x0) * (i + 0.5) / 4.0), m / 4.0); } }
static void loads_init(void)
{
    n_node = 0;
    /* OpenRocket component tree (rocket.ork), x from the nose tip */
    node(0.067, 0.015); node(0.0565, 0.030);                       /* nose shell, nose mass        */
    tube(0.10, 0.18, 0.040); node(0.14, 0.100);                    /* avionics bay, flight computer */
    tube(0.18, 0.26, 0.060); node(0.22, 0.050); node(0.21, 0.012); /* canard module, servos, canards */
    tube(0.24, 0.32, 0.090); node(0.3185, 0.008); node(0.28, 0.020); node(0.3335, 0.010); /* coupler */
    tube(0.26, 0.76, 0.141); node(0.51, 0.025); node(0.67, 0.004); /* body tube, chute, cord        */
    tube(0.76, 0.886, 0.010); tube(0.716, 0.886, 0.100);           /* fin can, engine block         */
    node(0.871, 0.003); node(0.854, 0.0075); node(0.769, 0.0075); node(0.85, 0.006); node(0.79, 0.005);
    /* calibrate: the OpenRocket total is 29.6 g heavier than the tree's
     * listed overrides (glue, fasteners, wiring); place it to hit the CG,
     * then scale and shift to the dispersed m and CG */
    double m = 0, mx = 0; for (int i = 0; i < n_node; i++) { m += nd_m0[i]; mx += nd_m0[i] * nd_x[i]; }
    if (C.m_dry > m) { node(((C.m_dry * C.x_cg_dry) - mx) / (C.m_dry - m), C.m_dry - m); }   /* nominal calibration */
    const double mt = C.m_dry * C.mass_scale, xt = C.x_cg_dry + C.cg_shift;
    m = 0; mx = 0; for (int i = 0; i < n_node; i++) { m += nd_m0[i]; mx += nd_m0[i] * nd_x[i]; }
    const double k = mt / m, dx = xt - (mx / m);
    ld_lump_It = 0;
    for (int i = 0; i < n_node; i++) { nd_m0[i] *= k; nd_x[i] += dx; ld_lump_It += nd_m0[i] * pow(nd_x[i] - xt, 2); }
    memset(LDM, 0, sizeof LDM); ld_resid_max = 0;
}
static void loads_eval(V3 Fsum, V3 wdot)
{
    double mm = MO.m[0]; if (P.ignited) { mm = interp(MO.t, MO.m, MO.n, P.t - P.t_ign); }
    const V3 fcg = scl(Fsum, 1.0 / P.m);
    const double xm0 = C.x_mot - 0.06, xm1 = C.L_total;
    /* the lumped It differs from the dynamics' (dispersed) It by a few %: scale
     * the rotational inertial term so that the cut aft of everything closes */
    double It_l = 0;
    for (int j = 0; j < n_node + 4; j++) {
        const double xj = (j < n_node) ? nd_x[j] : xm0 + ((xm1 - xm0) * ((j - n_node) + 0.5) / 4.0);
        It_l += ((j < n_node) ? nd_m0[j] : mm / 4.0) * pow(xj - P.xcg, 2);
    }
    const V3 wd = scl(wdot, P.It / fmax(It_l, 1e-6));
    double Pv[NLD + 1]; V3 Sv[NLD + 1], Mv[NLD + 1];
    for (int s = 0; s <= NLD; s++) {
        const double xs = (s < NLD) ? LD_X[s] : 10.0;
        const V3 rs = st(xs, 0, 0);
        V3 Fc = v3(0, 0, 0), Mc = v3(0, 0, 0);
        for (int i = 0; i < n_frc; i++) {
            if (fr_x[i] < xs) { Fc = add(Fc, fr_F[i]); Mc = add(Mc, crs(sub(st(fr_x[i], 0, 0), rs), fr_F[i])); }
        }
        for (int j = 0; j < n_node + 4; j++) {
            const double xj = (j < n_node) ? nd_x[j] : xm0 + ((xm1 - xm0) * ((j - n_node) + 0.5) / 4.0);
            const double mj = (j < n_node) ? nd_m0[j] : mm / 4.0;
            if (xj >= xs) { continue; }
            const V3 r = st(xj, 0, 0);
            const V3 fj = add(fcg, add(crs(wd, r), crs(P.w, crs(P.w, r))));
            const V3 Fi = scl(fj, -mj);
            Fc = add(Fc, Fi); Mc = add(Mc, crs(sub(r, rs), Fi));
        }
        Pv[s] = -Fc.z; Sv[s] = v3(Fc.x, Fc.y, 0); Mv[s] = v3(Mc.x, Mc.y, 0);
    }
    /* closure: a cut aft of everything must carry nothing */
    const double res = nrm(Mv[NLD]) / fmax(1e-3, P.m * G0 * C.L_total);
    if (res > ld_resid_max) { ld_resid_max = res;
        if (getenv("LD_DEBUG")) { fprintf(stderr, "t=%.3f res=%.3f V=%.1f wdot=(%.1f %.1f) w=(%.2f %.2f %.1f) pc=%.1f It=%.4f lump=%.4f M=(%.3f %.3f) F=(%.2f %.2f %.2f)\n",
            P.t - P.t_ign, res, P.V, wdot.x, wdot.y, P.w.x, P.w.y, P.w.z, P.pc, P.It, ld_lump_It, Mv[NLD].x, Mv[NLD].y, Fsum.x, Fsum.y, Fsum.z); } }
    const double R = 0.5 * C.d;
    for (int s = 0; s < NLD; s++) {
        const double Mb = nrm(Mv[s]), Pc = Pv[s], S = nrm(Sv[s]);
        if (Mb > LDM[s].Mb) { LDM[s].Mb = Mb; }
        if (Pc > LDM[s].P) { LDM[s].P = Pc; }
        if (-Pc > LDM[s].Pt) { LDM[s].Pt = -Pc; }
        if (S > LDM[s].S) { LDM[s].S = S; }
        const double Ne = fmax(Pc, 0.0) + (2.0 * Mb / R);       /* equivalent axial compression */
        if (Ne > LDM[s].Neq) { LDM[s].Neq = Ne; }
    }
}

static void plant_step(double dt)
{
    if (P.ignited) { P.t += dt; } else { P.t += dt; }
    mass_props();
    double rho, a_s; isa(C.site_elev + P.p.z, &rho, &a_s);
    const V3 wind = wind_at(P.p.z, dt);
    const V3 va_w = P.on_rail ? P.v : sub(P.v, wind);
    const V3 vb = qrot_t(P.q, va_w);
    const double V = nrm(vb);
    P.V = V; P.mach = V / a_s; P.qbar = 0.5 * rho * V * V;
    const double A = M_PI * C.d * C.d * 0.25;
    V3 F = v3(0, 0, 0), M = v3(0, 0, 0);
    double L_can = 0.0;                                         /* aero roll torque on the can */
    n_frc = 0;

    /* thrust (with misalignment) at the nozzle */
    const double T = thrust();
    if (T > 0) {
        const double e = C.thrust_mis_deg * D2R, az = C.thrust_az_deg * D2R;
        const V3 Fb = scl(v3(sin(e) * cos(az), sin(e) * sin(az), cos(e)), T);
        F = add(F, Fb); M = add(M, crs(st(C.L_total, 0, C.cg_off_y), Fb)); frc(C.L_total, Fb);
    }
    const int aero_on = (V > 1.0) && !P.landed;
    P.alpha = 0;
    if (aero_on) {
        const double q = P.qbar;
        const double Cd = C.Cd_scale * interp(AE.M, AE.Cd, AE.n, P.mach);
        const V3 Fdrag = scl(vb, -q * A * Cd / V);
        F = add(F, Fdrag);                                       /* drag along the air velocity */
        /* LD-01 A-L1: forebody 40 % at the nose, skin 30 % along the body, base 30 % */
        frc(0.05, scl(Fdrag, 0.4)); frc(C.L_total, scl(Fdrag, 0.3));
        for (int k = 0; k < 5; k++) { frc(0.15 + (0.15 * k), scl(Fdrag, 0.06)); }
        const V3 vlat = v3(vb.x, vb.y, 0.0);
        P.alpha = atan2(nrm(vlat), fmax(vb.z, 1e-6));
        /* body normal force (nose + rear fins + body), canards separately */
        const double CNa_tot = C.CNa_scale * interp(AE.M, AE.CNa, AE.n, P.mach);
        const double xcp_tot = interp(AE.M, AE.xcp, AE.n, P.mach) + C.cp_shift;
        const double CNa_can = 2.0 * C.kf_can / A;
        const double CNa_b = CNa_tot - CNa_can;
        const double xcp_b = ((xcp_tot * CNa_tot) - (C.x_canard * CNa_can)) / CNa_b;
        const double sa = fmin(sin(P.alpha), 1.0);
        const V3 dir = (nrm(vlat) > 1e-9) ? scl(vlat, 1.0 / nrm(vlat)) : v3(0, 0, 0);
        const V3 Fn = scl(dir, -q * A * CNa_b * sa);
        F = add(F, Fn); M = add(M, crs(st(xcp_b, 0, 0), Fn));
        {   /* split between nose and fins with the same total and moment */
            const double kf = (xcp_b - C.x_nose_cp) / (C.x_fin - C.x_nose_cp);
            frc(C.x_nose_cp, scl(Fn, 1.0 - kf)); frc(C.x_fin, scl(Fn, kf));
        }
        /* pitch damping: nose and rear fins see alpha_local = w x r */
        const double xs[2] = { C.x_nose_cp, C.x_fin }, cn[2] = { C.CNa_nose, C.CNa_fins };
        for (int i = 0; i < 2; i++) {
            const V3 r = st(xs[i], 0, 0);
            const V3 ul = crs(P.w, r); const V3 ulat = v3(ul.x, ul.y, 0);
            const V3 Fd = scl(ulat, -q * A * C.CNa_scale * cn[i] / V);
            F = add(F, Fd); M = add(M, crs(r, Fd)); frc(xs[i], Fd);
        }
        /* skin-friction roll damping on the forward body */
        M.z += -q * A * C.d * 2.0 * C.Cf * (C.L_body / C.d) * (P.w.z * C.d / (2.0 * V));
        M.z += q * A * C.d * C.cl0_body;
        /* canards, per fin: fin i at azimuth i*90 deg, tangential direction t_i */
        for (int i = 0; i < 4; i++) {
            const double th = i * M_PI * 0.5;
            const V3 r = st(C.x_canard, C.ycp_can * cos(th), C.ycp_can * sin(th));
            const V3 tv = v3(-sin(th), cos(th), 0.0);
            const V3 u = add(vb, crs(P.w, r));
            double al = P.delta[i] - (dot(u, tv) / fmax(u.z, 1.0));
            if (P.sv[i].fault == 4) { al = 0.0; }               /* unpowered: floats with the flow */
            const double ast = C.alpha_stall * D2R;
            al = fmax(fmin(al, ast), -ast);
            const V3 Fi = scl(tv, q * C.kf_can * C.Cld_scale * al);
            F = add(F, Fi); M = add(M, crs(r, Fi)); frc(C.x_canard, Fi);
        }
        /* rear fin can: net cant spins it, one misaligned fin rotates a side force */
        L_can = q * A * C.d * ((C.Cld_f * C.can_cant_deg * D2R) + (C.Clp_f * C.Clp_scale * P.pc * C.d / (2.0 * V)));
        if (C.fin1_mis_deg != 0.0) {
            const V3 tv = v3(-sin(P.phic), cos(P.phic), 0.0);
            const V3 Fs = scl(tv, q * C.kff_fin * C.fin1_mis_deg * D2R);
            F = add(F, Fs); M = add(M, crs(st(C.x_fin, 0, 0), Fs)); frc(C.x_fin, Fs);
            L_can += q * C.kff_fin * C.fin1_mis_deg * D2R * (C.d * 0.5 + 0.027);
        }
    }
    /* bearing: Coulomb (smoothed) + viscous between can and body */
    const double slip = P.pc - P.w.z;
    const double Tb = (C.bear_coul * tanh(slip / 0.5)) + (C.bear_visc * slip);
    M.z += Tb; L_can -= Tb;

    /* parachute on a shock cord from the attach station: drag of the canopy
     * acts along the cord; inflation over 0.4 s */
    if (P.chute) {
        P.chute_open = fmin(1.0, (P.t - P.t_chute) / 0.4);
        if ((P.t_ground > 0) && (P.t - P.t_ground > C.collapse_t)) {          /* canopy collapses on the ground */
            P.chute_open *= fmax(0.0, 1.0 - (P.t - P.t_ground - C.collapse_t));
        }
        const V3 vc = sub(P.v, wind);
        const double Vc = nrm(vc);
        if (Vc > 0.1) {
            /* canopy swing (pendulum ~0.4 Hz) and breathing: a Gauss-Markov tilt of
             * the drag line, sigma C.canopy_swing deg, and +/-10 % area pulsing */
            const double as = exp(-dt / 0.6), ss = C.canopy_swing * D2R * sqrt(1.0 - (as * as));
            P.swing_x = (as * P.swing_x) + (ss * nrand()); P.swing_y = (as * P.swing_y) + (ss * nrand());
            const double br = 1.0 + (0.1 * sin(2.0 * M_PI * 1.1 * P.t));
            V3 dirc = scl(vc, -1.0 / Vc);
            dirc = add(dirc, v3(P.swing_x, P.swing_y, 0.0));
            const V3 Fw = scl(dirc, 0.5 * rho * Vc * Vc * C.chute_cd_a * P.chute_open * br);
            const V3 Fb = qrot_t(P.q, Fw);
            F = add(F, Fb); M = add(M, crs(st(C.x_attach, 0, 0), Fb));
            /* body aero damping under the canopy (tumbling body) */
            /* scaled by each axis' inertia: time constant ~0.5 s at 5 m/s, and
             * explicit-Euler stable at dt_desc for any deployment speed */
            const double kd = 0.33 * rho * Vc;
            M = add(M, v3(-kd * P.It * P.w.x, -kd * P.It * P.w.y, -kd * P.Ixx * P.w.z));
        }
    }
    /* world-frame acceleration */
    V3 Fw = qrot(P.q, F);
    V3 acc = add(scl(Fw, 1.0 / P.m), v3(0, 0, -G0));
    V3 wdot = v3(0, 0, 0);
    if (P.on_rail) {
        const V3 zr = qrot(P.q, v3(0, 0, 1));
        double along = dot(acc, zr); if (!P.ignited || (along < 0)) { along = 0; }
        acc = scl(zr, along);
        if (P.ignited && (nrm(P.p) >= C.rail_len)) { P.on_rail = 0; }
    } else if (!P.landed) {
        /* Euler with the can's angular momentum about body z (gyroscopic) */
        const V3 I = v3(P.It, P.It, P.Ixx);
        const V3 H = v3(I.x * P.w.x, I.y * P.w.y, (I.z * P.w.z) + (C.I_can * P.pc));
        const V3 gyro = crs(P.w, H);
        wdot = v3((M.x - gyro.x) / I.x, (M.y - gyro.y) / I.y, (M.z - gyro.z) / I.z);
        if (P.ignited && !P.chute) { loads_eval(F, wdot); }
    }
    /* ground */
    if (P.ignited && !P.on_rail && (P.p.z < 0.0) && (P.t - P.t_ign > 2.0)) {
        if (!P.landed && (P.t_ground <= 0)) { P.t_ground = P.t; }
        const double fz = (C.ground_k * -P.p.z) - (C.ground_c * P.v.z);
        V3 Fg = v3(-C.ground_c * P.v.x, -C.ground_c * P.v.y, fmax(fz, 0.0));
        acc = add(acc, scl(Fg, 1.0 / P.m));
        wdot = add(wdot, scl(P.w, -40.0));
        if ((nrm(P.v) < 0.02) && (nrm(P.w) < 0.01) && (P.t - P.t_ground > 0.5)) {
            P.landed = 1; P.v = v3(0, 0, 0); P.w = v3(0, 0, 0); if (P.t_rest <= 0) { P.t_rest = P.t; }
        }
        const double g_imp = nrm(acc) / G0; if (g_imp > P.imp_g) { P.imp_g = g_imp; }
    }
    if (P.landed) { acc = v3(0, 0, 0); wdot = v3(0, 0, 0); P.v = v3(0, 0, 0); P.w = v3(0, 0, 0); }
    /* integrate (semi-implicit Euler) */
    P.v = add(P.v, scl(acc, dt));
    if (P.landed) { P.v = v3(0, 0, 0); }
    P.p = add(P.p, scl(P.v, dt));
    P.w = add(P.w, scl(wdot, dt));
    P.wdot = wdot;
    if (!P.on_rail) {
        P.pc += (L_can / C.I_can) * dt;
        if (P.landed) { P.pc = 0; }
        P.phic += (P.pc - P.w.z) * dt;
        P.q = qint(P.q, P.w, dt);
    }
    /* specific force at the IMU (lever arm) in the body frame */
    const V3 r_imu = st(C.x_imu, C.r_imu, 0);
    V3 fb = qrot_t(P.q, sub(acc, v3(0, 0, -G0)));
    if (!P.on_rail) { fb = add(fb, add(crs(wdot, r_imu), crs(P.w, crs(P.w, r_imu)))); }
    P.f_spec = fb;
    /* events */
    if (P.ignited) {
        if (P.p.z > P.apogee) { P.apogee = P.p.z; P.t_apogee = P.t - P.t_ign; }
        if (P.chute && (P.p.z < 20.0) && (P.p.z > 10.0)) { P.v_desc = -P.v.z; }
        if (!P.chute && (P.t_apogee > 0) && (P.v.z < 0) && (P.t - P.t_ign > P.t_apogee + C.chute_delay)) {
            P.chute = 1; P.t_chute = P.t;
            {   /* deployment state for the REC-01 recovery sub-sim */
                double rho_d, a_d; isa(C.site_elev + P.p.z, &rho_d, &a_d); (void)rho_d; (void)a_d;
                const V3 va = sub(P.v, wind);
                P.dep_V = nrm(va); P.dep_gam = atan2(va.z, sqrt((va.x * va.x) + (va.y * va.y))) * R2D;
                P.dep_h = P.p.z; P.dep_w = sqrt((P.w.x * P.w.x) + (P.w.y * P.w.y)) * R2D;
                const V3 ax = qrot(P.q, v3(0, 0, 1));
                P.dep_aoa = acos(fmax(-1.0, fmin(1.0, dot(ax, va) / fmax(P.dep_V, 1e-6)))) * R2D;
            }
        }
    }
}

/* ------------------------------------------------------------- firmware -- */
static Flight F;
static double ev_t[16]; static int ev_seen[16]; static int phase_bad;
static Phase last_phase = PH_BOOT;
static int phase_slot(Phase p)
{
    switch (p) {
    case PH_SETTLING: return 1; case PH_READY: return 2; case PH_BOOST: return 3; case PH_COAST: return 4;
    case PH_DESCENT: return 5; case PH_LANDED: return 6; case PH_LOCKOUT: return 7; case PH_CATO_SAFE: return 8;
    default: return 0;
    }
}
static void on_ev(void *ctx, EventCode c, uint32_t a, const char *t)
{
    (void)ctx; (void)t;
    if (c == EV_PHASE) {
        const int k = phase_slot((Phase)a);
        if (!ev_seen[k]) { ev_seen[k] = 1; ev_t[k] = P.t - P.t_ign; }
        if (((Phase)a == PH_SETTLING) && (last_phase == PH_READY)) { phase_bad++; }
        last_phase = (Phase)a;
    }
}

int main(int argc, char **argv)
{
    cfg_defaults();
    for (int i = 1; i < argc; i++) {
        char k[64]; const char *eq = strchr(argv[i], '=');
        if (!eq) { fprintf(stderr, "bad arg %s\n", argv[i]); return 2; }
        snprintf(k, sizeof k, "%.*s", (int)(eq - argv[i]), argv[i]); cfg_set(k, eq + 1);
    }
    seed_rng(C.seed);
    load_motor(C.motor); load_aero(C.aero);
    memset(&P, 0, sizeof P);
    P.on_rail = 1;
    {   /* rail attitude */
        const double tl = C.rail_tilt * D2R, az = C.rail_az * D2R;
        /* tilt the body z (nose) toward azimuth az (from north, clockwise) */
        const V3 ax = v3(-cos(az), sin(az), 0.0);     /* rotation axis horizontal, perpendicular to tilt */
        P.q.w = cos(tl / 2); P.q.x = ax.x * sin(tl / 2); P.q.y = ax.y * sin(tl / 2); P.q.z = 0;
    }
    imu_init();
    for (int i = 0; i < 4; i++) { servo_init(&P.sv[i], C.dt_asc); }
    mass_props(); loads_init();

    static const NavCfg NC = {
        .h_site_m = NAV_SITE_ELEV_M, .sat_mps2 = NAV_SAT_MPS2, .sat_budget_n = NAV_SAT_BUDGET_N, .still_acc_mps2 = NAV_STILL_ACC_MPS2,
        .still_gyro_radps = NAV_STILL_GYRO_RADPS, .acc_psd = NAV_ACC_PSD, .gps_var_m2 = NAV_GPS_VAR_M2,
        .gps_nis_max = NAV_GPS_NIS_MAX, .gps_rej_reset = NAV_GPS_REJ_RESET, .gps_v_var = NAV_GPS_V_VAR,
        .q_speed_min = NAV_Q_SPEED_MIN };
    static const DetectCfg DC = {
        .liftoff_acc = DET_LIFTOFF_ACC, .liftoff_n = DET_LIFTOFF_N, .liftoff_v = DET_LIFTOFF_V_MPS,
        .liftoff_n_nov = DET_LIFTOFF_N_NOV, .t_boost_min = DET_T_BOOST_MIN_S, .burnout_acc = DET_BURNOUT_ACC,
        .burnout_n = DET_BURNOUT_N, .apogee_n = DET_APOGEE_N, .t_apogee_max = DET_T_APOGEE_MAX_S,
        .t_land_min = DET_T_LAND_MIN_S, .t_land_max = DET_T_LAND_MAX_S, .land_acc_tol = DET_LAND_ACC_TOL,
        .land_gyro = DET_LAND_GYRO_RADPS, .land_v = DET_LAND_V_MPS, .land_n = DET_LAND_N,
        .land_tau = DET_LAND_TAU_S, .land_still_sd = DET_LAND_STILL_SD, .land_still_gyro = DET_LAND_STILL_GYRO,
        .land_shock_acc = DET_LAND_SHOCK_ACC, .land_n_shock = DET_LAND_N_SHOCK, .land_n_still = DET_LAND_N_STILL };
    Flight_Init(&F, &NC, &DC, MS_TO_FRAMES(READY_HOLD_MS), NAV_BRIDGE_N, false, on_ev, NULL);
    F.shadow = (C.shadow != 0);

    FILE *tr = NULL;
    if (C.trace[0]) {
        tr = fopen(C.trace, "w");
        fprintf(tr, "t,phase,h,vz,qbar,mach,alpha_deg,p_dps,q_dps,r_dps,pcan_dps,delta0,delta1,delta2,delta3,"
                    "fin0,h_est,vz_est,qbar_est,fx,fy,fz,trim,ax_imu,ay_imu,az_imu\n");
    }
    /* boot: 1 s gyro calibration + accel seed on the pad (as app.c boot()) */
    const double frame_dt = 1.0 / FRAME_HZ;
    double gsum[3] = { 0, 0, 0 }, asum[3] = { 0, 0, 0 }; int nb = 0;
    double t_frame = 0.0, t_cmd_apply[4] = { 0 }; (void)t_cmd_apply;
    double cmd_live[4] = { 0, 0, 0, 0 }, cmd_queued[4] = { 0 }, t_queue = -1;
    for (double tb = 0; tb < 1.0; tb += C.dt_desc) {
        plant_step(C.dt_desc); imu_advance(P.t, C.dt_desc, P.f_spec, P.w);
        if (tb >= t_frame) { for (int k = 0; k < 3; k++) { gsum[k] += IM.out_g[k]; asum[k] += IM.out_a[k]; } nb++; t_frame += frame_dt; }
    }
    float gbias[3], a0[3]; const float m0[3] = { 0, 0, 0 };
    for (int k = 0; k < 3; k++) { gbias[k] = (float)(gsum[k] / nb); a0[k] = (float)(asum[k] / nb); }
    IEKF_Seed(&F.iekf, a0, m0);
    Flight_GoPhase(&F, PH_CALIBRATE); Flight_GoPhase(&F, PH_SETTLING);

    /* metrics */
    double p_max_boost = 0, p_max_coast = 0, p2_coast = 0, n_coast = 0, aoa_max = 0, wlat_max = 0, q_max = 0;
    double defl_max = 0, sat_n = 0, act_n = 0, qhat_e2 = 0, qhat_emax = 0, n_q = 0, pcan_max = 0, ratio_max = 0;
    double nlat_max = 0, qalpha_max = 0, t_ready = -1, p_apo = 0, h_err_apo = 0, trim_last = 0, wl_coast_max = 0;
    double p_coast_end = 0, mach_max = 0, body_ratio_max = 0;
    int frames = 0; t_frame = P.t;
    double t_next_frame = P.t;
    while (P.t < C.t_max + C.pad_hold + 60) {
        const double dt = (P.ignited && !P.chute) ? C.dt_asc : C.dt_desc;
        /* servo command: applied sv_extra after the frame (compute + pulse width read) */
        if ((t_queue >= 0) && (P.t >= t_queue)) { memcpy(cmd_live, cmd_queued, sizeof cmd_live); t_queue = -1; }
        for (int i = 0; i < 4; i++) {
            if ((i == C.fault_fin) && P.ignited && (P.t - P.t_ign >= C.fault_t) && (P.sv[i].fault == 0)) {
                P.sv[i].fault = !strcmp(C.fault_kind, "stuck") ? 1 : !strcmp(C.fault_kind, "hardover") ? 2 :
                                !strcmp(C.fault_kind, "slow") ? 3 : !strcmp(C.fault_kind, "float") ? 4 :
                                !strcmp(C.fault_kind, "offset") ? 5 : 0;
            }
            P.delta[i] = P.sv[i].nd ? servo_step(&P.sv[i], cmd_live[i], dt) : 0.0;
        }
        plant_step(dt);
        /* IMU vibration level by regime */
        if (P.ignited && !P.landed && (P.t_ground <= 0)) {
            if (thrust() > 0) { IM.vib_g = C.vib_boost_g; IM.vib_res_g = C.vib_res_g; IM.vib_gyro = C.vib_gyro_dps; }
            else if (P.chute) { IM.vib_g = C.vib_chute_g; IM.vib_res_g = 0; IM.vib_gyro = 0.2; }
            else { const double k = fmin(P.qbar / 6000.0, 1.0);
                   IM.vib_g = C.vib_coast_g * k; IM.vib_res_g = 0.3 * C.vib_res_g * k; IM.vib_gyro = 0.3 * C.vib_gyro_dps * k; }
        } else { IM.vib_g = 0; IM.vib_res_g = 0; IM.vib_gyro = 0; }
        imu_advance(P.t, dt, P.f_spec, P.w);

        if (P.t + 1e-9 >= t_next_frame) {
            t_next_frame += frame_dt; frames++;
            FlightIn in; memset(&in, 0, sizeof in);
            in.imu_ok = true;
            for (int k = 0; k < 3; k++) { in.accel[k] = IM.out_a[k]; in.gyro_dps[k] = IM.out_g[k] - gbias[k]; }
            in.loop_us = 400; in.now_ms = (uint32_t)(P.t * 1000.0);
            /* GPS: 1 Hz altitude, lost under thrust */
            static double t_gps = 0;
            if (C.gps_on && (P.t >= t_gps)) {
                t_gps += 1.0;
                in.gps_enabled = true; in.gps_alive = true; in.gps_new = true;
                in.gps_good = !(P.ignited && (thrust() > 0));
                in.gps_alt_msl = (float)(C.site_elev + P.p.z + (C.gps_sd * nrand()));
            } else if (C.gps_on) { in.gps_enabled = true; in.gps_alive = true; }
            Flight_Step(&F, &in, (float)frame_dt);
            for (int i = 0; i < 4; i++) { cmd_queued[i] = F.fin[i] * C.fin_deg * D2R; }
            t_queue = P.t + C.sv_extra;
            const Phase ph = Flight_Phase(&F);
            if ((ph == PH_READY) && (t_ready < 0)) { t_ready = P.t; }
            if ((t_ready >= 0) && !P.ignited && (P.t - t_ready >= C.pad_hold)) { P.ignited = 1; P.t_ign = P.t; }
            /* metrics in flight (truth) */
            if (P.ignited && !P.chute) {
                const double tt = P.t - P.t_ign;
                const double pz = fabs(P.w.z) * R2D;
                const double wl = sqrt((P.w.x * P.w.x) + (P.w.y * P.w.y)) * R2D;
                if (!P.on_rail) {
                    if (thrust() > 0) { if (pz > p_max_boost) { p_max_boost = pz; } }
                    else if (P.v.z > 0) {
                        if (pz > p_max_coast) { p_max_coast = pz; } p2_coast += pz * pz; n_coast++;
                        if (wl > wl_coast_max) { wl_coast_max = wl; } p_coast_end = P.w.z * R2D;
                    }
                    if ((P.v.z > 0) && (P.qbar > 500.0) && (P.alpha * R2D > aoa_max)) { aoa_max = P.alpha * R2D; }
                    if (wl > wlat_max) { wlat_max = wl; }
                    if (P.qbar > q_max) { q_max = P.qbar; }
                    if (P.mach > mach_max) { mach_max = P.mach; }
                    const double qa = P.qbar * P.alpha * R2D; if ((P.v.z > 0) && (qa > qalpha_max)) { qalpha_max = qa; }
                    const double nl = sqrt((P.f_spec.x * P.f_spec.x) + (P.f_spec.y * P.f_spec.y)) / G0;
                    if (nl > nlat_max) { nlat_max = nl; }
                    const double pc = fabs(P.pc) * R2D; if (pc > pcan_max) { pcan_max = pc; }
                    /* can spin vs pitch natural frequency (CNa*q*A*SM/It) */
                    const double A = M_PI * C.d * C.d / 4.0;
                    const double xcp = interp(AE.M, AE.xcp, AE.n, P.mach);
                    const double kk = C.CNa_scale * interp(AE.M, AE.CNa, AE.n, P.mach) * P.qbar * A * (xcp - P.xcg) / P.It;
                    if ((kk > 0) && (P.qbar > 300)) {
                        const double r = fabs(P.pc) / sqrt(kk); if (r > ratio_max) { ratio_max = r; }
                        const double rb = fabs(P.w.z) / sqrt(kk); if (rb > body_ratio_max) { body_ratio_max = rb; }
                    }
                    for (int i = 0; i < 4; i++) {
                        const double dd = fabs(P.delta[i]) * R2D; if (dd > defl_max) { defl_max = dd; }
                        act_n++; if (fabs(F.fin_cmd[i] * C.fin_deg * D2R) >= 0.999 * C.sv_travel) { sat_n++; }
                    }
                    if ((P.qbar > 2000) && (tt > 0.1)) {
                        const double e = (F.nav.qbar - P.qbar) / P.qbar * 100.0;
                        qhat_e2 += e * e; n_q++; if (fabs(e) > qhat_emax) { qhat_emax = fabs(e); }
                    }
                }
                if ((P.t_apogee > 0) && (fabs(tt - P.t_apogee) < 0.5 * frame_dt)) {
                    p_apo = P.w.z * R2D; h_err_apo = Nav_Height(&F.nav) - P.p.z;
                }
                trim_last = F.ctrl.trim;
            }
            if (tr && ((frames % 4) == 0)) {
                fprintf(tr, "%.4f,%u,%.2f,%.2f,%.0f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.0f,%.2f,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f\n",
                        P.ignited ? P.t - P.t_ign : -(P.t), Phase_Index(ph), P.p.z, P.v.z, P.qbar, P.mach, P.alpha * R2D,
                        P.w.z * R2D, P.w.x * R2D, P.w.y * R2D, P.pc * R2D,
                        P.delta[0] * R2D, P.delta[1] * R2D, P.delta[2] * R2D, P.delta[3] * R2D, F.fin[0],
                        Nav_Height(&F.nav), Nav_VUp(&F.nav), F.nav.qbar, P.f_spec.x, P.f_spec.y, P.f_spec.z, F.ctrl.trim,
                        IM.out_a[0], IM.out_a[1], IM.out_a[2]);
            }
            if ((ph == PH_LANDED) && P.landed && (P.t - P.t_rest > 1.0)) { break; }
            if (P.ignited && (P.t - P.t_ign > C.t_max)) { break; }
            if ((ph == PH_LOCKOUT) && !P.ignited) { break; }
        }
    }
    if (tr) { fclose(tr); }
    const double tl_true = 0.0;
    printf("t_ground=%.2f seed=%llu motor=%s apogee=%.1f t_apogee=%.2f q_max=%.0f mach_max=%.3f "
           "det_liftoff=%.3f det_burnout=%.3f t_burnout=%.3f det_apogee=%.3f det_landed=%.2f t_rest=%.2f "
           "ok_seq=%d phase_bad=%d p_max_boost=%.1f p_max_coast=%.1f p_rms_coast=%.1f p_coast_end=%.1f p_apogee=%.1f "
           "wlat_max=%.1f wlat_coast_max=%.1f aoa_max=%.2f qalpha_max=%.0f nlat_max=%.2f defl_max=%.2f sat_frac=%.4f "
           "qhat_rms=%.2f qhat_max=%.2f h_err_apo=%.2f pcan_max=%.0f can_pitch_ratio=%.2f impact_g=%.1f trim=%.4f "
           "reached_ready=%d final_phase=%u body_pitch_ratio=%.2f\n",
           (P.t_ground > 0) ? P.t_ground - P.t_ign : -1, (unsigned long long)C.seed, C.motor, P.apogee, P.t_apogee, q_max, mach_max,
           ev_seen[3] ? ev_t[3] - tl_true : -1, ev_seen[4] ? ev_t[4] : -1, MO.tb,
           ev_seen[5] ? ev_t[5] : -1, ev_seen[6] ? ev_t[6] : -1, (P.t_rest > 0) ? P.t_rest - P.t_ign : -1,
           ev_seen[3] && ev_seen[4] && ev_seen[5] && ev_seen[6] && !ev_seen[7] && !ev_seen[8], phase_bad,
           p_max_boost, p_max_coast, (n_coast > 0) ? sqrt(p2_coast / n_coast) : 0, p_coast_end, p_apo,
           wlat_max, wl_coast_max, aoa_max, qalpha_max, nlat_max, defl_max, (act_n > 0) ? sat_n / act_n : 0,
           (n_q > 0) ? sqrt(qhat_e2 / n_q) : 0, qhat_emax, h_err_apo, pcan_max, ratio_max, P.imp_g, trim_last,
           t_ready >= 0, Phase_Index(Flight_Phase(&F)), body_ratio_max);
    printf("ld_resid=%.4f ld_lump_It=%.4f dep_V=%.2f dep_gam=%.1f dep_h=%.1f dep_aoa=%.1f dep_w=%.1f v_desc=%.2f", ld_resid_max, ld_lump_It,
           P.dep_V, P.dep_gam, P.dep_h, P.dep_aoa, P.dep_w, P.v_desc);
    for (int s = 0; s < NLD; s++) {
        printf(" ldM_%s=%.3f ldP_%s=%.2f ldT_%s=%.2f ldS_%s=%.2f ldN_%s=%.1f", LD_NAME[s], LDM[s].Mb, LD_NAME[s], LDM[s].P,
               LD_NAME[s], LDM[s].Pt, LD_NAME[s], LDM[s].S, LD_NAME[s], LDM[s].Neq);
    }
    printf("\n");
    return 0;
}
