/* ============================================================================
 * nav.c  —  see nav.h (NAV-01)
 * ============================================================================ */
#include "nav.h"
#include "so3_math.h"
#include <math.h>
#include <string.h>

#define G_STD          9.80665f
#define G_AVG_MAX_N    4096u           /* pad gravity average saturates here   */
#define GPS_AVG_MAX_N  600u            /* ~10 min of 1 Hz pad fixes            */
#define ISA_T0         288.15f         /* K                                    */
#define ISA_P0         101325.0f       /* Pa                                   */
#define ISA_LAPSE      0.0065f         /* K/m                                  */
#define ISA_R          287.053f        /* J/(kg K)                             */
#define ISA_EXP        5.255877f       /* g0/(R L)                             */
#define ISA_H_MAX      11000.0f        /* troposphere only                     */
#define P_PAD_H        0.01f           /* m^2   pad covariance after zeroing   */
#define P_PAD_V        0.0001f         /* (m/s)^2                              */

float Nav_IsaDensity(float h_msl_m)
{
    float h = h_msl_m;
    if (h < -500.0f) { h = -500.0f; }
    if (h > ISA_H_MAX) { h = ISA_H_MAX; }
    const float T = ISA_T0 - (ISA_LAPSE * h);
    const float p = ISA_P0 * powf(T / ISA_T0, ISA_EXP);
    return p / (ISA_R * T);
}

static void pad_zero(Nav *n)
{
    v3_set(n->p, 0.0f, 0.0f, 0.0f);
    v3_set(n->v, 0.0f, 0.0f, 0.0f);
    n->P[0] = P_PAD_H; n->P[1] = 0.0f; n->P[2] = 0.0f; n->P[3] = P_PAD_V;
}

void Nav_Init(Nav *n, const NavCfg *c)
{
    memset(n, 0, sizeof *n);
    n->c = *c;
    n->g_ref = G_STD;
    n->h_site = c->h_site_m;
    pad_zero(n);
}

static void update_q(Nav *n)
{
    n->speed = v3_norm(n->v);
    n->rho = Nav_IsaDensity(n->h_site + n->p[2]);
    n->qbar = (n->speed >= n->c.q_speed_min) ? (0.5f * n->rho * n->speed * n->speed) : 0.0f;
}

void Nav_Step(Nav *n, const float q[4], const float f[3], const float w[3], float dt, bool on_pad)
{
    if (!(dt > 0.0f)) { return; }
    float R[9], fn[3];
    q_to_R(q, R);
    m3_mulvec(R, f, fn);                               /* specific force, ENU */

    const float fmag = v3_norm(f);
    const bool clipped = (fabsf(f[0]) >= n->c.sat_mps2) || (fabsf(f[1]) >= n->c.sat_mps2)
                      || (fabsf(f[2]) >= n->c.sat_mps2);
    const bool still = (fabsf(fmag - n->g_ref) < n->c.still_acc_mps2)
                    && (v3_norm(w) < n->c.still_gyro_radps) && !clipped;

    if (on_pad && still) {
        /* Stationary on the pad: refresh the gravity reference, hold zero. */
        if (n->g_n < G_AVG_MAX_N) { n->g_sum += fn[2]; n->g_n++; }
        n->g_ref = n->g_sum / (float)n->g_n;
        n->flags |= NAV_F_INIT | NAV_F_STILL;
        n->flags &= ~(NAV_F_SAT | NAV_F_GPS_AIDED);
        pad_zero(n);
        n->a_up = 0.0f;
        update_q(n);
        return;
    }
    n->flags &= ~NAV_F_STILL;
    if (clipped) { n->flags |= NAV_F_SAT; }

    float a[3];
    v3_copy(fn, a);
    a[2] -= n->g_ref;
    n->a_up = a[2];
    for (uint32_t k = 0u; k < 3u; k++) {
        const float v0 = n->v[k];
        n->v[k] = v0 + (a[k] * dt);
        n->p[k] += 0.5f * (v0 + n->v[k]) * dt;
    }
    /* Vertical covariance: F = [1 dt; 0 1], Q from white accel noise. */
    const float q0 = n->c.acc_psd;
    const float P00 = n->P[0] + (dt * (n->P[1] + n->P[2])) + (dt * dt * n->P[3])
                    + (q0 * dt * dt * dt / 3.0f);
    const float P01 = n->P[1] + (dt * n->P[3]) + (q0 * dt * dt * 0.5f);
    const float P11 = n->P[3] + (q0 * dt);
    n->P[0] = P00; n->P[1] = P01; n->P[2] = P01; n->P[3] = P11;
    update_q(n);
}

void Nav_PadGps(Nav *n, float alt_msl_m)
{
    if (!isfinite(alt_msl_m) || (n->gps_n >= GPS_AVG_MAX_N)) { return; }
    n->gps_sum += alt_msl_m;
    n->gps_n++;
    n->h_site = n->gps_sum / (float)n->gps_n;
    n->flags |= NAV_F_GPS_SITE;
}

bool Nav_GpsAlt(Nav *n, float alt_msl_m)
{
    if (!isfinite(alt_msl_m)) { return false; }
    const float y = (alt_msl_m - n->h_site) - n->p[2];   /* innovation, m */
    const float S = n->P[0] + n->c.gps_var_m2;
    const bool gated = !(S > 0.0f) || ((y * y) / S > n->c.gps_nis_max);
    if (((n->flags & NAV_F_SAT) != 0u) || (gated && (n->gps_rej + 1u >= n->c.gps_rej_reset))) {
        /* Re-anchor: the inertial vertical channel is not credible. */
        n->p[2] = alt_msl_m - n->h_site;
        n->v[2] = 0.0f;
        n->P[0] = n->c.gps_var_m2; n->P[1] = 0.0f; n->P[2] = 0.0f; n->P[3] = n->c.gps_v_var;
        n->gps_rej = 0u;
        n->flags |= NAV_F_GPS_AIDED;
        n->flags &= ~NAV_F_SAT;      /* vertical now GPS-anchored (q_hat irrelevant post-apogee) */
        update_q(n);
        return true;
    }
    if (gated) {
        n->gps_rej++;
        n->flags &= ~NAV_F_GPS_AIDED;
        return false;
    }
    n->gps_rej = 0u;
    const float k0 = n->P[0] / S, k1 = n->P[2] / S;
    n->p[2] += k0 * y;
    n->v[2] += k1 * y;
    const float P00 = (1.0f - k0) * n->P[0];
    const float P01 = (1.0f - k0) * n->P[1];
    const float P11 = n->P[3] - (k1 * n->P[1]);
    n->P[0] = P00; n->P[1] = P01; n->P[2] = P01; n->P[3] = P11;
    n->flags |= NAV_F_GPS_AIDED;
    update_q(n);
    return true;
}

float Nav_Height(const Nav *n) { return n->p[2]; }
float Nav_VUp(const Nav *n)    { return n->v[2]; }
bool  Nav_QValid(const Nav *n) { return ((n->flags & NAV_F_INIT) != 0u) && ((n->flags & NAV_F_SAT) == 0u); }
