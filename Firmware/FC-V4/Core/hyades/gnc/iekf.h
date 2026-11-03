/* ============================================================================
 * iekf.h  —  Left-Invariant EKF: attitude + gyro bias
 *
 * State (nominal)   : q [w x y z] (body->world, Hamilton), b_gyro (rad/s, body)
 * Error state (6)   : [ d-theta (3) | d-bias (3) ],  body-frame (left-invariant)
 * Covariance        : P (6x6)
 *
 * WHY "INVARIANT" (and an honest note on scope)
 *   The error is defined multiplicatively in the body frame, q = q_hat (x)
 *   exp([d-theta]x). For the SO(3) propagation Ṙ = R[w]x this makes the error
 *   dynamics  d-theta_dot = -[w_hat]x d-theta - d-bias - n_g  — the propagation
 *   Jacobian depends only on the MEASURED rate w_hat, NOT on the attitude
 *   estimate. That trajectory-independence is the Invariant-EKF property
 *   (Barrau & Bonnabel 2017) and is what improves consistency under large
 *   attitude error / high rates.
 *
 *   For the ATTITUDE-ONLY problem with body-frame vector measurements (gravity
 *   from accel, field from mag), this mechanisation coincides with a correctly
 *   formulated body-frame MEKF; the dramatic IEKF gain (state-independent F for
 *   the velocity coupling) appears only on SE_2(3) with a velocity state, which
 *   needs a velocity/position aiding source (GPS is status-only here). Gyro bias also breaks
 *   exact invariance ("imperfect IEKF"). So this is the invariant FORM, with
 *   the consistency benefit it can deliver here, and the propagate/update/reset
 *   boundary isolated so the error geometry can be swapped later (EqF, SE_2(3)).
 *
 * Pure module: no I/O, no globals. Host-buildable for off-target unit tests.
 *
 * References
 *   Barrau, A. & Bonnabel, S. (2017) IEEE TAC 62(4), 1797-1812.
 *   Trawny, N. & Roumeliotis, S.I. (2005) Univ. Minnesota TR-2005-002 (Q, reset).
 *   Sola, J. (2017) arXiv:1711.02508 (error-state mechanics).
 * ============================================================================ */
#ifndef IEKF_H_
#define IEKF_H_

#include <stdint.h>
#include <stdbool.h>

#define IEKF_N 6

typedef struct {
    /* nominal state */
    float q[4];               /* [w x y z], body -> world (ENU)               */
    float bias[3];            /* gyro bias estimate (rad/s, body)             */
    float P[IEKF_N*IEKF_N];   /* error covariance                             */

    /* most recent bias-corrected body rate (rad/s) — for the controller.
     * Written by IEKF_Predict; saves the controller re-deriving it.          */
    float omega[3];

    /* process noise (continuous-time spectral densities) */
    float q_angle;            /* rad/s/sqrt(Hz)  (gyro angle random walk)     */
    float q_bias;             /* rad/s/sqrt(Hz)  (bias random walk)           */

    /* measurement noise (isotropic, direction-only / normalised updates) */
    float r_accel;            /* normalised                                   */
    float r_mag;              /* normalised                                   */

    /* magnitude gates */
    float accel_min, accel_max;   /* m/s^2                                    */
    float mag_min,   mag_max;     /* Gauss                                    */

    /* innovation (NIS) gate, chi-square(3); <=0 disables */
    float nis_gate;

    /* world reference directions (ENU). Stored as unit vectors. */
    float grav_ref[3];        /* "up" (specific force at rest points to +up)  */
    float mag_ref[3];         /* local field direction                        */
} IEKF_State;

/* Initialise state + parameters from board.h. Sets q = identity, bias = 0,
 * P = diag(P0). grav_ref/mag_ref loaded from board.h (mag_ref refined by the
 * seed below). */
void IEKF_Init(IEKF_State *s);

/* Seed attitude from a settled accel+mag pair (BODY frame). Sets roll/pitch
 * from gravity and yaw from the tilt-compensated field, and refines mag_ref to
 * the seed-consistent world direction (zero initial mag innovation). */
void IEKF_Seed(IEKF_State *s, const float accel_ms2[3], const float mag_gauss[3]);

/* Gyro propagation. omega_dps: BODY rates in deg/s (static bias already removed
 * by the driver); dt seconds. Stores the bias-corrected rate in s->omega. */
void IEKF_Predict(IEKF_State *s, const float omega_dps[3], float dt);

/* Vector measurement updates (BODY frame). Return true if applied, false if
 * gated by magnitude or rejected by the NIS test. */
bool IEKF_UpdateAccel(IEKF_State *s, const float accel_ms2[3]);
bool IEKF_UpdateMag  (IEKF_State *s, const float mag_gauss[3]);

/* Convenience: Euler angles (degrees, ZYX) for telemetry/navball. */
void IEKF_GetEuler(const IEKF_State *s, float *roll, float *pitch, float *yaw);

/* Health inputs for FDIR: every state and covariance element finite, and the
 * TILT error variance (rad^2): trace(P_theta) minus the component about
 * gravity, which is unobservable without a magnetometer. */
bool  IEKF_IsFinite(const IEKF_State *s);
float IEKF_TiltVar(const IEKF_State *s);

#endif /* IEKF_H_ */
