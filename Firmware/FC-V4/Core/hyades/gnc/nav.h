/* ============================================================================
 * nav.h  —  Vertical navigation and dynamic-pressure estimate (NAV-01)
 *
 * Strapdown velocity/position in a pad-fixed ENU frame, driven by the IEKF
 * attitude and the measured specific force. No barometer: height comes from
 * integration alone, with a GPS altitude update allowed only after apogee.
 * The output that matters for control is q_hat = 1/2 rho(h) |v|^2, used to
 * schedule the roll gain (Kp = c/q_hat, Matlab roll-tuning design).
 *
 *   v_dot = R(q) f_b - g_ref e_up        p_dot = v         (trapezoidal)
 *
 * g_ref is the gravity magnitude MEASURED on the pad (mean of the nav-frame
 * up component of R f while stationary), not 9.80665: this cancels the
 * accelerometer's vertical bias and scale error at 1 g, which would otherwise
 * integrate to ~0.1 m/s per mg-second.
 *
 * Pad behaviour (Nav_Step with on_pad): the state integrates every frame but
 * is zeroed whenever the sample looks stationary (| |f| - g_ref | and |w|
 * small). Liftoff is therefore integrated from its FIRST high-g sample, not
 * from the end of the detector's persistence window (50 ms at 20 g would be
 * a 10 m/s error).
 *
 * Saturation: any accel axis at the sensor's range marks the velocity (and
 * so h, q_hat) unreliable until the next pad reset (NAV_F_SAT). The QMI8658A
 * and the H743's ICM-42688-P both clip at +/-16 g; NAV-01 limits the motor to
 * 14 g peak.
 *
 * GPS: Nav_GpsAlt is a 2-state [h, v_up] Kalman update with an NIS gate. The
 * caller decides when it may run (DESCENT/LANDED only: the receiver loses lock
 * under boost and 1 Hz is too slow for the q_hat that matters). If the
 * inertial solution is known bad (saturated) or the gate has rejected
 * gps_rej_reset fixes in a row, the vertical channel is RE-ANCHORED to the
 * fix (h = GPS, v_up = 0, covariance reset) instead of gating forever -- a
 * gate alone would lock out the only aiding source exactly when it matters.
 *
 * Accuracy is set by the accelerometer scale factor through boost (NAV-01
 * error budget): ~2 % SF on 250 m/s burnout = ~5 m/s, ~4 % in q_hat.
 *
 * Pure module: no I/O, no globals. Units SI. Frames as board.h §1.
 * ============================================================================ */
#ifndef NAV_H_
#define NAV_H_

#include <stdint.h>
#include <stdbool.h>

#define NAV_F_INIT      (1u << 0)   /* gravity reference measured on the pad    */
#define NAV_F_SAT       (1u << 1)   /* accel clipped since the pad: v/h/q bad    */
#define NAV_F_GPS_AIDED (1u << 2)   /* last GPS altitude update was accepted    */
#define NAV_F_GPS_SITE  (1u << 3)   /* site elevation from the GPS pad average  */
#define NAV_F_STILL     (1u << 4)   /* last pad sample judged stationary        */

typedef struct {
    float h_site_m;        /* default pad elevation above MSL (config)          */
    float sat_mps2;        /* |f| per axis at/above this = clipped              */
    float still_acc_mps2;  /* | |f| - g_ref | below this = stationary candidate  */
    float still_gyro_radps;/* |w| below this = stationary candidate             */
    float acc_psd;         /* white accel PSD for the vertical KF, (m/s^2)^2/Hz  */
    float gps_var_m2;      /* GPS altitude variance                             */
    float gps_nis_max;     /* chi^2(1) gate on the altitude innovation          */
    uint16_t gps_rej_reset;/* this many consecutive gate rejections -> re-anchor */
    float gps_v_var;       /* (m/s)^2 v_up variance after a re-anchor            */
    float q_speed_min;     /* below this speed q_hat is reported as 0           */
} NavCfg;

typedef struct {
    NavCfg   c;
    float    p[3], v[3];   /* ENU relative to the pad origin (m, m/s)           */
    float    P[4];         /* [h, v_up] covariance, row-major 2x2               */
    float    g_ref;        /* pad-measured gravity (m/s^2)                       */
    float    g_sum;
    uint32_t g_n;
    float    gps_sum;      /* pad GPS altitude average -> h_site                */
    uint32_t gps_n;
    uint16_t gps_rej;      /* consecutive NIS rejections                        */
    float    h_site;       /* in use (m MSL)                                    */
    float    a_up;         /* last nav-frame up acceleration (m/s^2)            */
    float    speed, rho, qbar;
    uint32_t flags;
} Nav;

void  Nav_Init(Nav *n, const NavCfg *c);

/* One frame. q: IEKF body->world quaternion [w x y z]; f: body specific force
 * (m/s^2); w: body rate (rad/s); on_pad: SETTLING/READY (stationary zeroing on). */
void  Nav_Step(Nav *n, const float q[4], const float f[3], const float w[3], float dt, bool on_pad);

/* Pad only: accumulate a valid GPS altitude fix into the site elevation. */
void  Nav_PadGps(Nav *n, float alt_msl_m);

/* After apogee only (caller's rule): fuse a GPS altitude. Returns accepted. */
bool  Nav_GpsAlt(Nav *n, float alt_msl_m);

float Nav_Height(const Nav *n);        /* above the pad, m                   */
float Nav_VUp(const Nav *n);           /* m/s, + up                          */
bool  Nav_QValid(const Nav *n);        /* q_hat usable for scheduling        */

/* ISA troposphere density (kg/m^3) at h m MSL, standard sea-level day. */
float Nav_IsaDensity(float h_msl_m);

#endif /* NAV_H_ */
