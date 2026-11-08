/* ============================================================================
 * detect.c  —  see detect.h
 * ============================================================================ */
#include "detect.h"
#include <math.h>
#include <string.h>

#define G_STD 9.80665f

void Detect_Init(Detect *d, const DetectCfg *c)
{
    memset(d, 0, sizeof *d);
    d->c = *c;
    d->last = PH_BOOT;
}

/* Persistence: count while cond holds, reset otherwise; true at n. */
static bool persist_c(uint16_t *cnt, bool cond, uint16_t n)
{
    *cnt = cond ? (uint16_t)((*cnt < 0xFFFFu) ? (*cnt + 1u) : *cnt) : 0u;
    return *cnt >= n;
}
static bool persist(Detect *d, bool cond, uint16_t n) { return persist_c(&d->n, cond, n); }

/* Landing stillness statistics: exponentially weighted mean/variance. */
static bool still_update(Detect *d, const DetectIn *in, float dt)
{
    const DetectCfg *c = &d->c;
    if (!in->imu_ok) { return false; }
    if (!d->ewma_init) {
        d->f_mean = in->f_norm; d->f_var = c->land_still_sd * c->land_still_sd; d->w_mean = in->w_norm; d->ewma_init = true;
    }
    const float a = (c->land_tau > dt) ? (dt / c->land_tau) : 1.0f;
    const float e = in->f_norm - d->f_mean;
    d->f_mean += a * e;
    d->f_var = (1.0f - a) * (d->f_var + (a * e * e));
    d->w_mean += a * (in->w_norm - d->w_mean);
    return (d->f_var < (c->land_still_sd * c->land_still_sd)) && (d->w_mean < c->land_still_gyro)
        && (fabsf(d->f_mean - G_STD) < c->land_acc_tol);
}

bool Detect_LiftoffPending(const Detect *d) { return ((d->last == PH_READY) || (d->last == PH_SETTLING)) && (d->n > 0u); }

Phase Detect_Step(Detect *d, Phase now, const DetectIn *in, float dt)
{
    if (now != d->last) {
        d->last = now; d->n = 0u; d->n2 = 0u; d->t_ms = 0u; d->ewma_init = false; d->shock_seen = false;
    }
    const float dms = dt * 1000.0f;
    if ((dms > 0.0f) && (dms < 1000.0f) && (d->t_ms < 0xFFF00000u)) {   /* range check before the cast */
        d->t_ms += (uint32_t)(dms + 0.5f);
    }
    d->t = (float)d->t_ms * 0.001f;
    const DetectCfg *c = &d->c;
    Phase to = now;

    switch (now) {
    case PH_READY:
    case PH_SETTLING:      /* STM-01 F3: a launch that was not READY must still be sequenced */
        if (persist(d, in->f_axial > c->liftoff_acc, c->liftoff_n_nov)
            || ((d->n >= c->liftoff_n) && in->v_valid && (in->v_up > c->liftoff_v))) {
            to = PH_BOOST;
        }
        break;
    case PH_BOOST:
        if ((d->t >= c->t_boost_min) && persist(d, in->f_axial < c->burnout_acc, c->burnout_n)) {
            to = PH_COAST;
        }
        break;
    case PH_COAST:
        if (persist(d, in->v_valid && (in->v_up < 0.0f), c->apogee_n) || (d->t >= c->t_apogee_max)) {
            to = PH_DESCENT;
        }
        break;
    case PH_DESCENT:
    case PH_CATO_SAFE: {   /* STM-01 F2: CATO must reach LANDED (log close, beacon) */
        const bool quiet = in->imu_ok && (fabsf(in->f_norm - G_STD) < c->land_acc_tol)
                        && (in->w_norm < c->land_gyro) && in->v_gps && (fabsf(in->v_up) < c->land_v);
        const bool armed = d->t >= c->t_land_min;
        const bool shock = in->imu_ok && (in->f_norm > c->land_shock_acc);
        if (armed && shock) {
            d->shock_seen = true;
            d->ewma_init = false;      /* stillness statistics restart after the impact */
        }
        const bool still = !shock && still_update(d, in, dt);
        /* One stillness counter; the shock only shortens the window needed. */
        (void)persist_c(&d->n2, armed && still, 0xFFFFu);
        const uint16_t need = d->shock_seen ? c->land_n_shock : c->land_n_still;
        const bool gps_rule = persist(d, armed && quiet, c->land_n);
        if (gps_rule || ((need > 0u) && (d->n2 >= need)) || (d->t >= c->t_land_max)) {
            to = PH_LANDED;
        }
        break;
    }
    default:
        d->n = 0u;
        break;
    }
    return to;
}
