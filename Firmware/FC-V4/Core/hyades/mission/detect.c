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
static bool persist(Detect *d, bool cond, uint16_t n)
{
    d->n = cond ? (uint16_t)((d->n < 0xFFFFu) ? (d->n + 1u) : d->n) : 0u;
    return d->n >= n;
}

bool Detect_LiftoffPending(const Detect *d) { return (d->last == PH_READY) && (d->n > 0u); }

Phase Detect_Step(Detect *d, Phase now, const DetectIn *in, float dt)
{
    if (now != d->last) { d->last = now; d->n = 0u; d->t_ms = 0u; }
    const float dms = dt * 1000.0f;
    if ((dms > 0.0f) && (dms < 1000.0f) && (d->t_ms < 0xFFF00000u)) {   /* range check before the cast */
        d->t_ms += (uint32_t)(dms + 0.5f);
    }
    d->t = (float)d->t_ms * 0.001f;
    const DetectCfg *c = &d->c;
    Phase to = now;

    switch (now) {
    case PH_READY:
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
    case PH_DESCENT: {
        const bool quiet = (fabsf(in->f_norm - G_STD) < c->land_acc_tol) && (in->w_norm < c->land_gyro)
                        && in->v_gps && (fabsf(in->v_up) < c->land_v);
        if (((d->t >= c->t_land_min) && persist(d, quiet, c->land_n)) || (d->t >= c->t_land_max)) {
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
