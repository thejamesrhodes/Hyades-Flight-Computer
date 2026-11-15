/* ============================================================================
 * alloc.c  —  Canard control allocation (see alloc.h, ALLOC-01)
 * ============================================================================ */
#include "alloc.h"
#include <math.h>

#define ALLOC_FIN_DEFLECT_RAD (FIN_DEFLECT_DEG * 0.017453292519943295f)
#define ALLOC_DITHER_NORM     (CTRL_DITHER_AMP_RAD / ALLOC_FIN_DEFLECT_RAD)
#define ALLOC_TIE_EPS         1.0e-4f   /* ticks: fractional parts closer than this are a tie */
#define ALLOC_MAX_TICKS       ((int32_t)(ALLOC_TICKS_PER_NORM + 0.5f))   /* = Alloc_NormToTicks(1) */

static float clamp_sym(float x, float lim)
{
    return (x > lim) ? lim : ((x < -lim) ? -lim : x);
}

#if CTRL_ALLOC_STAGGER
/* Tie order for the extra ticks: an opposite pair first (pure roll), the odd
 * tick alternating between the pair's two fins on alternate frames. */
static const uint8_t k_order[2][4] = { { 0u, 2u, 1u, 3u }, { 2u, 0u, 3u, 1u } };

static int32_t clamp_ticks(int32_t t)
{
    return (t > ALLOC_MAX_TICKS) ? ALLOC_MAX_TICKS : ((t < -ALLOC_MAX_TICKS) ? -ALLOC_MAX_TICKS : t);
}
#endif

void Alloc_Init(Alloc *a)
{
    a->k = 0u;
    a->dither = 0.0f;
    for (uint32_t i = 0u; i < 4u; i++) { a->v[i] = 0.0f; a->tick[i] = 0; }
}

int32_t Alloc_NormToTicks(float norm)
{
    float x = isfinite(norm) ? norm : 0.0f;
    x = clamp_sym(x, 1.0f);
    /* |x| <= 1, so |x * ALLOC_TICKS_PER_NORM| <= 2483: in int32 range (std A2) */
    return (int32_t)lroundf(x * ALLOC_TICKS_PER_NORM);
}

float Alloc_TicksToNorm(int32_t ticks)
{
    return (float)ticks / ALLOC_TICKS_PER_NORM;
}

void Alloc_Apply(Alloc *a, const float cmd[4], bool active, bool dither, float out[4])
{
    if (!active) {
        for (uint32_t i = 0u; i < 4u; i++) { out[i] = cmd[i]; a->tick[i] = Alloc_NormToTicks(cmd[i]); }
        a->k = 0u;
        a->dither = 0.0f;
        return;
    }

    /* 1. cruciform virtual axes */
    float r = 0.25f * (cmd[0] + cmd[1] + cmd[2] + cmd[3]);
    float p = 0.5f * (cmd[0] - cmd[2]);
    float y = 0.5f * (cmd[1] - cmd[3]);
    float n = 0.25f * ((cmd[0] + cmd[2]) - (cmd[1] + cmd[3]));

    /* 2. null-space dither: top/bottom +a, left/right -a. Each servo still sees
     * +/-a about its command; the canard forces cancel in pairs and the roll
     * contributions cancel between the pairs. */
#if CTRL_DITHER_ON && CTRL_DITHER_NULL
    a->dither = !dither ? 0.0f : ((((a->k / (CTRL_DITHER_PERIOD_N / 2u)) & 1u) == 0u) ? ALLOC_DITHER_NORM : -ALLOC_DITHER_NORM);
    n += a->dither;
#else
    (void)dither;
    a->dither = 0.0f;
#endif

    /* 3. priority saturation: roll, then pitch/yaw (direction kept), then null */
    r = clamp_sym(r, CTRL_FIN_LIMIT);
    const float head = CTRL_FIN_LIMIT - fabsf(r);
    float m = fmaxf(fabsf(p), fabsf(y));
    if (m > head) {
        const float s = (m > 0.0f) ? (head / m) : 0.0f;
        p *= s; y *= s; m = head;
    }
    n = clamp_sym(n, head - m);
    a->v[0] = r; a->v[1] = p; a->v[2] = y; a->v[3] = n;

    const float u[4] = { (r + p) + n, (r + y) - n, (r - p) + n, (r - y) - n };

#if CTRL_ALLOC_STAGGER
    /* 4. sum-preserving quantisation */
    int32_t base[4];
    float frac[4];
    float sum = 0.0f;
    int32_t bsum = 0;
    for (uint32_t i = 0u; i < 4u; i++) {
        const float t = u[i] * ALLOC_TICKS_PER_NORM;     /* |u| <= 1 by step 3 */
        base[i] = (int32_t)floorf(t);
        frac[i] = t - (float)base[i];
        sum += t;
        bsum += base[i];
    }
    int32_t extra = (int32_t)lroundf(sum) - bsum;        /* 0..4: frac in [0, 1) */
    if (extra < 0) { extra = 0; }
    if (extra > 4) { extra = 4; }
    const uint8_t *ord = k_order[a->k & 1u];
    bool used[4] = { false, false, false, false };
    for (int32_t j = 0; j < extra; j++) {
        int32_t best = -1;
        for (uint32_t q = 0u; q < 4u; q++) {
            const uint8_t i = ord[q];
            if (used[i]) { continue; }
            if ((best < 0) || (frac[i] > (frac[best] + ALLOC_TIE_EPS))) { best = (int32_t)i; }
        }
        if (best >= 0) { base[best] += 1; used[best] = true; }
    }
    for (uint32_t i = 0u; i < 4u; i++) {
        a->tick[i] = clamp_ticks(base[i]);
        out[i] = Alloc_TicksToNorm(a->tick[i]);
    }
#else
    for (uint32_t i = 0u; i < 4u; i++) { out[i] = u[i]; a->tick[i] = Alloc_NormToTicks(u[i]); }
#endif
    a->k++;
}
