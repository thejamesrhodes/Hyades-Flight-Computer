/* ============================================================================
 * iekf.c  —  Left-Invariant EKF (attitude + gyro bias)
 *
 * Sensors / noise
 *   Gyro  L3GD20    : eta_g = 0.03 dps/rtHz = 5.236e-4 rad/s/rtHz  -> q_angle
 *                     bias random walk -> q_bias  (board.h)
 *   Accel LSM303    : gravity-direction update (quasi-static), magnitude-gated
 *   Mag   LSM303    : field-direction update, magnitude-gated
 *
 * Error dynamics (derivation, body-frame / left-invariant error):
 *   q = q_hat (x) exp([d-theta]x),  w_hat = w_meas - b_hat
 *     d-theta_dot = -[w_hat]x d-theta - d-bias - n_g
 *     d-bias_dot  =  n_b
 *   => F = [ -[w_hat]x  -I ;  0  0 ]   (depends on w_hat only: invariant)
 *
 * Measurement (body observation of a world-fixed unit direction d):
 *   h(q) = R(q)^T d,   with the left-invariant error
 *     h ≈ R_hat^T d + [R_hat^T d]x d-theta
 *   => innovation nu = y_unit - y_hat,  H = [ [y_hat]x | 0 ],  y_hat = R_hat^T d
 *
 * Covariance update: Joseph form (Bierman 1977) for guaranteed symmetry/PSD.
 * Discrete process noise Q_d: Trawny & Roumeliotis (2005) attitude+bias model,
 * including the angle<->bias cross-covariance and the dt^3 angle term.
 * ============================================================================ */
#include "iekf.h"
#include "board.h"
#include "so3_math.h"

/* ----------------------------------------------------------------------------
 * Init
 * -------------------------------------------------------------------------- */
void IEKF_Init(IEKF_State *s)
{
    s->q[0]=1.0f; s->q[1]=0.0f; s->q[2]=0.0f; s->q[3]=0.0f;
    s->bias[0]=s->bias[1]=s->bias[2]=0.0f;
    s->omega[0]=s->omega[1]=s->omega[2]=0.0f;

    matN_zero(s->P, IEKF_N);
    for (int i=0;i<3;i++) { s->P[i*IEKF_N+i]       = IEKF_P0_ANGLE; }
    for (int i=3;i<6;i++) { s->P[i*IEKF_N+i]       = IEKF_P0_BIAS;  }

    s->q_angle = IEKF_Q_ANGLE;
    s->q_bias  = IEKF_Q_BIAS;
    s->r_accel = IEKF_R_ACCEL;
    s->r_mag   = IEKF_R_MAG;
    s->accel_min = IEKF_ACCEL_MIN; s->accel_max = IEKF_ACCEL_MAX;
    s->mag_min   = IEKF_MAG_MIN;   s->mag_max   = IEKF_MAG_MAX;
    s->nis_gate  = IEKF_NIS_GATE;

    /* world references as unit vectors */
    s->grav_ref[0]=BOARD_GRAV_REF_E; s->grav_ref[1]=BOARD_GRAV_REF_N; s->grav_ref[2]=BOARD_GRAV_REF_U;
    (void)v3_normalize(s->grav_ref);
    s->mag_ref[0]=BOARD_MAG_REF_E; s->mag_ref[1]=BOARD_MAG_REF_N; s->mag_ref[2]=BOARD_MAG_REF_U;
    (void)v3_normalize(s->mag_ref);
}

/* ----------------------------------------------------------------------------
 * Seed attitude from accel + mag (tilt-compensated compass, ST AN3192).
 * Body Z is the longitudinal axis (board.h). At rest the body specific force
 * points to +Up. We seed roll/pitch from gravity and yaw from the horizontal
 * field, then refine mag_ref = R_seed * m_body so the first mag update has zero
 * innovation (consistent world field, declination-agnostic).
 * -------------------------------------------------------------------------- */
void IEKF_Seed(IEKF_State *s, const float accel_ms2[3], const float mag_gauss[3])
{
    float a[3]; v3_copy(accel_ms2, a);
    if (v3_normalize(a) < 1e-6f) { return; }

    /* roll about X, pitch about Y, from the gravity direction (a ~ +up) */
    float roll  = atan2f(a[1], a[2]);
    float pitch = atan2f(-a[0], sqrtf(a[1]*a[1] + a[2]*a[2]));

    float cr=cosf(roll),  sr=sinf(roll);
    float cp=cosf(pitch), sp=sinf(pitch);

    /* tilt-correct the field into the horizontal plane (AN3192 eq. 14-15) */
    float mx=mag_gauss[0], my=mag_gauss[1], mz=mag_gauss[2];
    float by =  my*cr - mz*sr;
    float bx =  mx*cp + my*sr*sp + mz*cr*sp;
    float yaw = atan2f(-by, bx);

    /* ZYX Euler -> quaternion (Hamilton) */
    float cy=cosf(0.5f*yaw),   sy=sinf(0.5f*yaw);
    float cp2=cosf(0.5f*pitch), sp2=sinf(0.5f*pitch);
    float cr2=cosf(0.5f*roll),  sr2=sinf(0.5f*roll);
    s->q[0]=cr2*cp2*cy + sr2*sp2*sy;
    s->q[1]=sr2*cp2*cy - cr2*sp2*sy;
    s->q[2]=cr2*sp2*cy + sr2*cp2*sy;
    s->q[3]=cr2*cp2*sy - sr2*sp2*cy;
    q_normalize(s->q);

    /* refine world field: mag_ref = R_seed * m_body (unit) */
    float m[3]; v3_copy(mag_gauss, m);
    if (v3_normalize(m) > 1e-6f) {
        float R[9]; q_to_R(s->q, R);
        float ref[3]; m3_mulvec(R, m, ref);
        if (v3_normalize(ref) > 1e-6f) { v3_copy(ref, s->mag_ref); }
    }
}

/* ----------------------------------------------------------------------------
 * Predict
 * -------------------------------------------------------------------------- */
void IEKF_Predict(IEKF_State *s, const float omega_dps[3], float dt)
{
    const float d2r = 0.017453292519943295f;

    /* bias-corrected body rate (rad/s); cache for the controller */
    float w[3] = {
        omega_dps[0]*d2r - s->bias[0],
        omega_dps[1]*d2r - s->bias[1],
        omega_dps[2]*d2r - s->bias[2]
    };
    v3_copy(w, s->omega);

    /* nominal attitude: exact exp-map step  q <- q (x) exp(w*dt) */
    float rv[3] = { w[0]*dt, w[1]*dt, w[2]*dt };
    float dq[4]; q_from_rotvec(rv, dq);
    float qn[4]; q_mul(s->q, dq, qn); q_normalize(qn);
    s->q[0]=qn[0]; s->q[1]=qn[1]; s->q[2]=qn[2]; s->q[3]=qn[3];

    /* State-transition Phi (2nd order). Top-left Theta, top-right Psi. */
    float W[9], W2[9];
    m3_skew(w, W);
    m3_mul(W, W, W2);
    float dt2 = dt*dt;

    float Phi[IEKF_N*IEKF_N];
    matN_identity(Phi, IEKF_N);
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        /* Theta = I - W*dt + 0.5*W^2*dt^2 (identity diagonal already set) */
        Phi[i*IEKF_N+j] = ((i==j)?1.0f:0.0f) - W[i*3+j]*dt + 0.5f*W2[i*3+j]*dt2;
        /* Psi = -(I*dt - 0.5*W*dt^2) */
        Phi[i*IEKF_N+(j+3)] = -(((i==j)?dt:0.0f) - 0.5f*W[i*3+j]*dt2);
    }

    /* Discrete process noise (Trawny & Roumeliotis 2005) */
    float qa = s->q_angle*s->q_angle;
    float qb = s->q_bias *s->q_bias;
    float dt3 = dt2*dt;
    float Q[IEKF_N*IEKF_N];
    matN_zero(Q, IEKF_N);
    float q_tt = qa*dt + qb*dt3/3.0f;   /* angle  */
    float q_tb = -0.5f*qb*dt2;          /* angle<->bias cross */
    float q_bb = qb*dt;                 /* bias   */
    for (int i=0;i<3;i++) {
        Q[i*IEKF_N+i]           = q_tt;
        Q[i*IEKF_N+(i+3)]       = q_tb;
        Q[(i+3)*IEKF_N+i]       = q_tb;
        Q[(i+3)*IEKF_N+(i+3)]   = q_bb;
    }

    /* P <- Phi P Phi^T + Q */
    float PhiP[IEKF_N*IEKF_N], PhiT[IEKF_N*IEKF_N], tmp[IEKF_N*IEKF_N];
    matN_mul(Phi, s->P, PhiP, IEKF_N);
    matN_transpose(Phi, PhiT, IEKF_N);
    matN_mul(PhiP, PhiT, tmp, IEKF_N);
    matN_add(tmp, Q, s->P, IEKF_N);
    matN_symmetrize(s->P, IEKF_N);
}

/* ----------------------------------------------------------------------------
 * Generic vector measurement update.
 *   ref_world : unit world direction (already normalised)
 *   meas_body : measured body vector (raw scale; normalised here)
 *   R_noise   : isotropic measurement variance (normalised units)
 * Returns false if the NIS gate rejects the update.
 * -------------------------------------------------------------------------- */
static bool iekf_update_vec(IEKF_State *s, const float ref_world[3],
                            const float meas_body[3], float R_noise)
{
    float R[9]; q_to_R(s->q, R);

    /* predicted body direction y_hat = R^T * ref */
    float yhat[3]; m3_T_mulvec(R, ref_world, yhat);
    (void)v3_normalize(yhat);

    /* measured unit direction */
    float ymeas[3]; v3_copy(meas_body, ymeas);
    if (v3_normalize(ymeas) < 1e-9f) { return false; }

    /* innovation */
    float nu[3]; v3_sub(ymeas, yhat, nu);

    /* H = [ [y_hat]x | 0 ]  (3 x 6, row-major) */
    float H[3*IEKF_N];
    for (int i=0;i<3*IEKF_N;i++) { H[i]=0.0f; }
    float Sk[9]; m3_skew(yhat, Sk);
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) { H[i*IEKF_N+j] = Sk[i*3+j]; }

    /* HP = H P  (3 x 6) */
    float HP[3*IEKF_N];
    for (int i=0;i<3;i++) for (int j=0;j<IEKF_N;j++) {
        float acc=0.0f;
        for (int k=0;k<IEKF_N;k++) { acc += H[i*IEKF_N+k]*s->P[k*IEKF_N+j]; }
        HP[i*IEKF_N+j]=acc;
    }
    /* S = HP H^T + R I  (3 x 3) */
    float S[9];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        float acc=0.0f;
        for (int k=0;k<IEKF_N;k++) { acc += HP[i*IEKF_N+k]*H[j*IEKF_N+k]; }
        S[i*3+j] = acc + ((i==j)?R_noise:0.0f);
    }
    float Sinv[9];
    if (!m3_inverse(S, Sinv)) { return false; }

    /* NIS gate: nu^T S^-1 nu ~ chi-square(3) */
    if (s->nis_gate > 0.0f) {
        float Sv[3]; m3_mulvec(Sinv, nu, Sv);
        float nis = v3_dot(nu, Sv);
        if (nis > s->nis_gate) { return false; }
    }

    /* K = P H^T S^-1  (6 x 3) */
    float PHT[IEKF_N*3];
    for (int i=0;i<IEKF_N;i++) for (int j=0;j<3;j++) {
        float acc=0.0f;
        for (int k=0;k<IEKF_N;k++) { acc += s->P[i*IEKF_N+k]*H[j*IEKF_N+k]; }
        PHT[i*3+j]=acc;
    }
    float K[IEKF_N*3];
    for (int i=0;i<IEKF_N;i++) for (int j=0;j<3;j++) {
        float acc=0.0f;
        for (int k=0;k<3;k++) { acc += PHT[i*3+k]*Sinv[k*3+j]; }
        K[i*3+j]=acc;
    }

    /* error state dx = K nu */
    float dx[IEKF_N];
    for (int i=0;i<IEKF_N;i++) {
        float acc=0.0f;
        for (int j=0;j<3;j++) { acc += K[i*3+j]*nu[j]; }
        dx[i]=acc;
    }

    /* reset: fold d-theta into q (exp map), add d-bias */
    float dqv[3] = { dx[0], dx[1], dx[2] };
    float dq[4]; q_from_rotvec(dqv, dq);
    float qn[4]; q_mul(s->q, dq, qn); q_normalize(qn);
    s->q[0]=qn[0]; s->q[1]=qn[1]; s->q[2]=qn[2]; s->q[3]=qn[3];
    s->bias[0]+=dx[3]; s->bias[1]+=dx[4]; s->bias[2]+=dx[5];

    /* Joseph form: P = (I-KH) P (I-KH)^T + K R K^T */
    float KH[IEKF_N*IEKF_N];
    for (int i=0;i<IEKF_N;i++) for (int j=0;j<IEKF_N;j++) {
        float acc=0.0f;
        for (int k=0;k<3;k++) { acc += K[i*3+k]*H[k*IEKF_N+j]; }
        KH[i*IEKF_N+j]=acc;
    }
    float IKH[IEKF_N*IEKF_N];
    matN_identity(IKH, IEKF_N);
    for (int i=0;i<IEKF_N*IEKF_N;i++) { IKH[i]-=KH[i]; }

    float IKHT[IEKF_N*IEKF_N], t1[IEKF_N*IEKF_N], t2[IEKF_N*IEKF_N];
    matN_transpose(IKH, IKHT, IEKF_N);
    matN_mul(IKH, s->P, t1, IEKF_N);
    matN_mul(t1, IKHT, t2, IEKF_N);
    /* + K R K^T  (R isotropic -> R * K K^T) */
    for (int i=0;i<IEKF_N;i++) for (int j=0;j<IEKF_N;j++) {
        float acc=0.0f;
        for (int k=0;k<3;k++) { acc += K[i*3+k]*K[j*3+k]; }
        t2[i*IEKF_N+j] += R_noise*acc;
    }
    for (int i=0;i<IEKF_N*IEKF_N;i++) { s->P[i]=t2[i]; }
    matN_symmetrize(s->P, IEKF_N);
    return true;
}

bool IEKF_UpdateAccel(IEKF_State *s, const float accel_ms2[3])
{
    float amag = v3_norm(accel_ms2);
    if (amag < s->accel_min || amag > s->accel_max) { return false; }
    return iekf_update_vec(s, s->grav_ref, accel_ms2, s->r_accel);
}

bool IEKF_UpdateMag(IEKF_State *s, const float mag_gauss[3])
{
    float mmag = v3_norm(mag_gauss);
    if (mmag < s->mag_min || mmag > s->mag_max) { return false; }
    return iekf_update_vec(s, s->mag_ref, mag_gauss, s->r_mag);
}

void IEKF_GetEuler(const IEKF_State *s, float *roll, float *pitch, float *yaw)
{
    q_to_euler_deg(s->q, roll, pitch, yaw);
}
