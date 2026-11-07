/* ============================================================================
 * control.h  —  Geometric SO(3) attitude (outer) + INDI rate loop (inner)
 *
 * OUTER  (Lee et al. 2010; integral per Goodarzi et al. 2013)
 *   Geometric attitude error e_R, e_Omega on SO(3) -> desired body angular
 *   acceleration wdot_des. Setpoint = identity (nose-up). No linearisation,
 *   globally valid. Rate term is the measured (bias-corrected) gyro fed back
 *   directly (not a differentiated setpoint).
 *
 * INNER  (Smeur et al. 2016, INDI)
 *   Realises wdot_des incrementally:
 *     delta_cmd = delta_f + G^+ (wdot_des - wdot_f)
 *   where wdot_f is the filtered measured angular acceleration (differentiated
 *   gyro) and delta_f is the filtered actuator estimate, BOTH through the SAME
 *   2nd-order low-pass so their delays match (the rule that makes INDI work).
 *   G^+ is the closed-form pseudoinverse of the cruciform effectiveness matrix;
 *   this single operation is also the control allocation.
 *
 * ROLL LAW (CTRL_ROLL_LAW). 0 = INDI as below. 1 = the Matlab roll-tuning
 * design (controller.m, Mode 1): delta = -Kp(q) p + trim, Kp = c/q_e with
 * q_e = clamp(q_hat, CTRL_ROLL_Q_MIN, ..), plus the bounded trim integrator
 * (dead band, rate limit, clamp, freeze on saturation). Dither is NOT carried
 * across: 125 Hz cannot be produced by a 100 Hz loop. Fins: all four take
 * delta / FIN_DEFLECT (roll common mode), limited to the canard travel.
 *
 * MODES (degraded-mode ladder, h743_flight_architecture 4.1). Each step down
 * depends on strictly fewer inputs; the mode is chosen by the fault arbiter:
 *   CTRL_MODE_FULL      geometric outer loop + INDI     (estimator + gyro)
 *   CTRL_MODE_RATE_DAMP fin = -K_rd * body rate, clipped (gyro only; survives
 *                       an estimator failure)
 *   CTRL_MODE_OFF       fins centred (passive stability; IMU lost / not armed)
 * Entering FULL is bumpless: the INDI actuator filter is preloaded with the
 * last command and the rate-difference reference with the current rate.
 *
 * PURE module: no I/O, no globals. Output is a normalised fin vector [-1,+1].
 * Host-testable in isolation (no estimator dependency).
 *
 * Fins (cruciform, looking down the nose, body Z = roll axis out the nose):
 *   fin[0]=top(0deg) fin[1]=right(90) fin[2]=bottom(180) fin[3]=left(270)
 *   top/bottom drive pitch(X); left/right drive yaw(Y); all four drive roll(Z).
 *
 * BENCH NOTE: with no aerodynamic torque the measured wdot does not respond to
 * fin motion, so a held attitude error makes the INDI increment accumulate and
 * the fins drive toward their limit (then recover when level). This is correct
 * INDI behaviour; in flight (or HIL with a plant model) the resulting torque
 * closes the loop and the increment settles.
 * ============================================================================ */
#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>
#include <stdbool.h>

/* 2nd-order biquad state (Direct Form II transposed), per scalar channel. */
typedef struct { float z1, z2; } Biquad;

typedef enum { CTRL_MODE_OFF = 0, CTRL_MODE_RATE_DAMP, CTRL_MODE_FULL } CtrlMode;

typedef struct {
    CtrlMode mode;            /* set via Ctrl_SetMode() only                  */
    float fin_last[4];        /* last output (for bumpless mode changes)      */

    /* q-scheduled roll law (CTRL_ROLL_LAW 1): dynamic pressure from NAV-01 */
    float qbar;               /* Pa, last value given to Ctrl_SetQbar         */
    bool  q_valid;            /* false -> CTRL_ROLL_Q_FALLBACK (low gain)     */
    float trim;               /* bounded trim integrator, rad at the canard   */

    /* outer-loop integral on the Lie algebra (anti-windup: norm-clamped) */
    float e_I[3];

    /* INDI state */
    float omega_prev[3];      /* for the angular-acceleration difference      */
    float delta_f[4];         /* filtered actuator estimate (per fin)         */
    Biquad bq_wdot[3];        /* filter for Omega_dot (body axes)             */
    Biquad bq_delta[4];       /* filter for the actuator (fins)               */

    /* shared low-pass coefficients (b0,b1,b2,a1,a2), a0 normalised out */
    float b0, b1, b2, a1, a2;
} Controller;

/* Initialise gains/filters from board.h. Starts in CTRL_MODE_OFF. */
void Ctrl_Init(Controller *c);

/* Change mode. No-op if unchanged; otherwise resets the integral and prepares
 * the INDI filters for a bumpless start from the last fin command. */
void Ctrl_SetMode(Controller *c, CtrlMode mode);

/* Dynamic pressure for the roll schedule (call before Ctrl_Update). A non-
 * finite or invalid value selects CTRL_ROLL_Q_FALLBACK, the envelope's
 * highest q, i.e. the lowest (always-stable, under-damped) gain. */
void Ctrl_SetQbar(Controller *c, float qbar_pa, bool valid);

/* q-scheduled roll law (exposed for tests): canard deflection command, rad,
 * from roll rate p (rad/s). trim_on selects the bounded trim integrator. */
float Ctrl_RollLaw(Controller *c, float p_radps, float dt, bool trim_on);

/* One control step.
 *   q              : attitude quaternion [w x y z], body->world (estimator)
 *   omega_radps    : bias-corrected body rate (rad/s) from the estimator (FULL)
 *   gyro_radps     : boot-bias-corrected raw gyro rate (rad/s)       (RATE_DAMP)
 *   dt             : seconds since last call
 *   fin_out[4]     : normalised fin commands [-1,+1]   (all zero in OFF)
 */
void Ctrl_Update(Controller *c, const float q[4], const float omega_radps[3],
                 const float gyro_radps[3], float dt, float fin_out[4]);

#endif /* CONTROL_H_ */
