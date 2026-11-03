/* ============================================================================
 * control.c  —  Geometric SO(3) outer loop + INDI inner loop (pure)
 *
 * References
 *   Lee, T., Leok, M. & McClamroch, N.H. (2010) Geometric tracking control on
 *     SE(3), 49th IEEE CDC, 5420-5425.
 *   Goodarzi, F., Lee, D. & Lee, T. (2013) Geometric nonlinear PID on SE(3),
 *     ECC, 3845-3850.
 *   Smeur, E.J.J., Chu, Q. & de Croon, G.C.H.E. (2016) Adaptive INDI for MAV
 *     attitude control, JGCD 39(3), 450-461.
 * ============================================================================ */
#include "control.h"
#include "board.h"
#include "so3_math.h"

/* Effectiveness pseudoinverse constants (closed form). G maps fin[-1,+1] ->
 * body angular accel [pitch(X), yaw(Y), roll(Z)]; for the cruciform G G^T is
 * diagonal so G^+ = G^T (G G^T)^-1 reduces to these scalars. */
#define INV_PITCH  (1.0f / (2.0f * CTRL_EFF_PITCH))
#define INV_YAW    (1.0f / (2.0f * CTRL_EFF_YAW))
#define INV_ROLL   (1.0f / (4.0f * CTRL_EFF_ROLL))

/* RBJ biquad low-pass (Butterworth Q = 1/sqrt(2)). Coefficients normalised so
 * a0 = 1. Shared by the Omega_dot and actuator filters (delay matching). */
static void biquad_design(Controller *c, float fc_hz, float fs_hz)
{
    const float two_pi = 6.283185307179586f;
    float w0 = two_pi * fc_hz / fs_hz;
    float cw = cosf(w0), sw = sinf(w0);
    float Q  = 0.70710678f;
    float alpha = sw / (2.0f * Q);

    float b0 = (1.0f - cw) * 0.5f;
    float b1 =  1.0f - cw;
    float b2 = (1.0f - cw) * 0.5f;
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cw;
    float a2 =  1.0f - alpha;

    float inv = 1.0f / a0;
    c->b0 = b0*inv; c->b1 = b1*inv; c->b2 = b2*inv;
    c->a1 = a1*inv; c->a2 = a2*inv;
}

static inline float biquad_step(const Controller *c, Biquad *s, float x)
{
    float y = c->b0 * x + s->z1;
    s->z1 = c->b1 * x - c->a1 * y + s->z2;
    s->z2 = c->b2 * x - c->a2 * y;
    return y;
}

/* Set a unity-DC-gain biquad's state to its steady state for a constant input
 * u, so the output starts at u (z1 = (1-b0)u, z2 = (b2-a2)u). */
static void biquad_preload(const Controller *c, Biquad *s, float u)
{
    s->z1 = (1.0f - c->b0) * u;
    s->z2 = (c->b2 - c->a2) * u;
}

static float clampf(float x, float lim)
{
    return (x > lim) ? lim : ((x < -lim) ? -lim : x);
}

void Ctrl_Init(Controller *c)
{
    c->mode = CTRL_MODE_OFF;
    for (int i=0;i<3;i++) { c->e_I[i]=0.0f; c->omega_prev[i]=0.0f; }
    for (int i=0;i<4;i++) { c->delta_f[i]=0.0f; c->fin_last[i]=0.0f; }
    for (int i=0;i<3;i++) { c->bq_wdot[i].z1=0.0f; c->bq_wdot[i].z2=0.0f; }
    for (int i=0;i<4;i++) { c->bq_delta[i].z1=0.0f; c->bq_delta[i].z2=0.0f; }
    biquad_design(c, CTRL_INDI_LPF_HZ, (float)FRAME_HZ);
}

void Ctrl_SetMode(Controller *c, CtrlMode mode)
{
    if (mode == c->mode) { return; }
    c->mode = mode;
    for (int i=0;i<3;i++) { c->e_I[i]=0.0f; biquad_preload(c, &c->bq_wdot[i], 0.0f); }
    for (int i=0;i<4;i++) {
        c->delta_f[i] = c->fin_last[i];
        biquad_preload(c, &c->bq_delta[i], c->fin_last[i]);
    }
}

/* Gyro-only rate damper: fin = -K * rate, per channel, clipped. */
static void rate_damp(const float w[3], float fin_out[4])
{
#if CTRL_ENABLE_PITCHYAW
    const float p = -CTRL_RD_GAIN_PITCHYAW * w[0];
    const float y = -CTRL_RD_GAIN_PITCHYAW * w[1];
#else
    const float p = 0.0f, y = 0.0f;
#endif
    const float r = -CTRL_RD_GAIN_ROLL * w[2];
    fin_out[0] = clampf( p + r, CTRL_FIN_LIMIT);   /* top    */
    fin_out[1] = clampf( y + r, CTRL_FIN_LIMIT);   /* right  */
    fin_out[2] = clampf(-p + r, CTRL_FIN_LIMIT);   /* bottom */
    fin_out[3] = clampf(-y + r, CTRL_FIN_LIMIT);   /* left   */
}

void Ctrl_Update(Controller *c, const float q[4], const float omega[3],
                 const float gyro[3], float dt, float fin_out[4])
{
    if (c->mode != CTRL_MODE_FULL) {
        if (c->mode == CTRL_MODE_RATE_DAMP) { rate_damp(gyro, fin_out); }
        else { fin_out[0]=fin_out[1]=fin_out[2]=fin_out[3]=0.0f; }
        /* Keep the rate-difference reference current for a bumpless FULL entry. */
        c->omega_prev[0]=omega[0]; c->omega_prev[1]=omega[1]; c->omega_prev[2]=omega[2];
        for (int i=0;i<4;i++) { c->fin_last[i]=fin_out[i]; }
        return;
    }
    if (dt < 1e-6f) { dt = 1e-6f; }

    /* ---- 1. measured angular acceleration (rad/s^2), filtered -------------- */
    float wdot_f[3];
    for (int i=0;i<3;i++) {
        float wdot = (omega[i] - c->omega_prev[i]) / dt;
        wdot_f[i]  = biquad_step(c, &c->bq_wdot[i], wdot);
    }
    c->omega_prev[0]=omega[0]; c->omega_prev[1]=omega[1]; c->omega_prev[2]=omega[2];

    /* ---- 2. geometric outer loop -> desired angular acceleration ----------- */
    float R[9]; q_to_R(q, R);

    /* attitude error vs identity setpoint: e_R = 0.5*vee(R_d^T R - R^T R_d),
     * R_d = I  =>  e_R = 0.5*vee(R - R^T)  (Lee 2010). Body axes X,Y,Z. */
    float e_R[3]; m3_vee_antisym(R, e_R);

    /* rate error (Omega_d = 0): e_W = omega (body rates, bias-corrected) */
    const float *e_W = omega;

    /* integral on the Lie algebra, norm-clamped */
    for (int i=0;i<3;i++) { c->e_I[i] += (e_R[i] + CTRL_KI_C * e_W[i]) * dt; }
    float ein = sqrtf(c->e_I[0]*c->e_I[0]+c->e_I[1]*c->e_I[1]+c->e_I[2]*c->e_I[2]);
    if (ein > CTRL_EI_LIMIT && ein > 1e-9f) {
        float s = CTRL_EI_LIMIT / ein;
        c->e_I[0]*=s; c->e_I[1]*=s; c->e_I[2]*=s;
    }

    /* per-axis gains: X,Y = pitch,yaw (PY); Z = roll */
    const float kR[3] = { CTRL_KR_PITCHYAW, CTRL_KR_PITCHYAW, CTRL_KR_ROLL };
    const float kW[3] = { CTRL_KW_PITCHYAW, CTRL_KW_PITCHYAW, CTRL_KW_ROLL };
    const float kI[3] = { CTRL_KI_PITCHYAW, CTRL_KI_PITCHYAW, CTRL_KI_ROLL };

    float wdot_des[3];
    for (int i=0;i<3;i++) {
        wdot_des[i] = -kR[i]*e_R[i] - kW[i]*e_W[i] - kI[i]*c->e_I[i];
    }

    /* ---- 3. INDI increment + cruciform allocation (G^+) -------------------- */
    float da[3] = { wdot_des[0]-wdot_f[0], wdot_des[1]-wdot_f[1], wdot_des[2]-wdot_f[2] };

#if CTRL_ENABLE_PITCHYAW
    float inc_p = da[0]*INV_PITCH;   /* pitch (X) */
    float inc_y = da[1]*INV_YAW;     /* yaw   (Y) */
#else
    /* Roll-only vehicle: pitch/yaw channels disabled entirely (no INDI
     * increments), so fins move only in roll common-mode. */
    float inc_p = 0.0f; (void)da[0];
    float inc_y = 0.0f; (void)da[1];
#endif
    float inc_r = da[2]*INV_ROLL;    /* roll  (Z), common to all fins */

    float dinc[4];
    dinc[0] =  inc_p + inc_r;   /* top    */
    dinc[1] =  inc_y + inc_r;   /* right  */
    dinc[2] = -inc_p + inc_r;   /* bottom */
    dinc[3] = -inc_y + inc_r;   /* left   */

    /* delta_cmd = delta_f + increment, saturated per fin (INDI anti-windup:
     * the SATURATED command is fed back into the actuator filter below). */
    for (int i=0;i<4;i++) {
        fin_out[i] = clampf(c->delta_f[i] + dinc[i], CTRL_FIN_LIMIT);
        c->fin_last[i] = fin_out[i];
    }

    /* ---- 4. update the filtered actuator estimate (delay-matched) ---------- */
    for (int i=0;i<4;i++) {
        c->delta_f[i] = biquad_step(c, &c->bq_delta[i], fin_out[i]);
    }
}
