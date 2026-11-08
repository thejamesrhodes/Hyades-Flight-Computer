/* ============================================================================
 * logrec.c  —  see logrec.h
 * ============================================================================ */
#include "logrec.h"
#include "crc.h"
#include <math.h>
#include <string.h>

static void put16(uint8_t *d, uint32_t o, uint16_t v) { d[o] = (uint8_t)v; d[o+1u] = (uint8_t)(v >> 8); }
static void put32(uint8_t *d, uint32_t o, uint32_t v)
{
    d[o] = (uint8_t)v; d[o+1u] = (uint8_t)(v >> 8); d[o+2u] = (uint8_t)(v >> 16); d[o+3u] = (uint8_t)(v >> 24);
}
static uint16_t get16(const uint8_t *d, uint32_t o) { return (uint16_t)(d[o] | ((uint16_t)d[o+1u] << 8)); }
static uint32_t get32(const uint8_t *d, uint32_t o)
{
    return (uint32_t)d[o] | ((uint32_t)d[o+1u] << 8) | ((uint32_t)d[o+2u] << 16) | ((uint32_t)d[o+3u] << 24);
}

/* Range-checked float -> int16 (A2: no out-of-range float->int conversion). */
static int16_t q16(float x, float scale)
{
    if (!isfinite(x)) { return INT16_MIN; }
    const float y = x * scale;
    if (y >=  32767.0f) { return  32767; }
    if (y <= -32767.0f) { return -32767; }
    return (int16_t)lroundf(y);
}
static int8_t q8(float x)
{
    if (!isfinite(x)) { return INT8_MIN; }
    const float y = x * 127.0f;
    if (y >=  127.0f) { return  127; }
    if (y <= -127.0f) { return -127; }
    return (int8_t)lroundf(y);
}

static void seal(uint8_t *r) { put16(r, LOGREC_SIZE - 2u, crc16_ccitt(r, LOGREC_SIZE - 2u)); }

void LogRec_PackState(uint8_t r[LOGREC_SIZE], const LogState *s)
{
    memset(r, 0, LOGREC_SIZE);
    r[0] = LOGREC_STATE; r[1] = s->phase; r[2] = s->mode; r[3] = s->flags;
    put32(r, 4u, s->t_us);
    put16(r, 8u, s->seq);
    for (uint32_t i = 0u; i < 4u; i++) { put16(r, 10u + 2u*i, (uint16_t)q16(s->q[i], 32767.0f)); }
    for (uint32_t i = 0u; i < 3u; i++) {
        put16(r, 18u + 2u*i, (uint16_t)q16(s->w_dps[i],    10.0f));
        put16(r, 24u + 2u*i, (uint16_t)q16(s->a_ms2[i],    100.0f));
    }
    put16(r, 30u, (uint16_t)q16(s->h_m, 10.0f));
    put16(r, 32u, (uint16_t)q16(s->v_up, 100.0f));
    {
        const float qh = isfinite(s->qbar_pa) ? (s->qbar_pa * 0.5f) : 0.0f;
        put16(r, 34u, (qh <= 0.0f) ? 0u : ((qh >= 65535.0f) ? 0xFFFFu : (uint16_t)lroundf(qh)));
    }
    put16(r, 44u, (uint16_t)q16(s->trim_rad, 1.0e4f));
    for (uint32_t i = 0u; i < 4u; i++) { r[36u + i] = (uint8_t)q8(s->fin[i]); }
    put16(r, 40u, s->loop_us);
    put16(r, 42u, s->faults);
    seal(r);
}

void LogRec_PackEvent(uint8_t r[LOGREC_SIZE], const LogEvent *e)
{
    memset(r, 0, LOGREC_SIZE);
    r[0] = LOGREC_EVENT; r[1] = e->code; r[2] = e->phase;
    put32(r, 4u, e->t_us);
    put16(r, 8u, e->seq);
    put32(r, 12u, e->arg0);
    put32(r, 16u, e->arg1);
    memcpy(&r[20], e->text, sizeof(e->text));
    seal(r);
}

void LogRec_PackBoot(uint8_t r[LOGREC_SIZE], const LogBoot *b)
{
    memset(r, 0, LOGREC_SIZE);
    r[0] = LOGREC_BOOT; r[1] = (uint8_t)LOGREC_VERSION;
    put16(r, 2u, b->frame_hz);
    put32(r, 4u, b->t_us);
    put16(r, 8u, b->seq);
    put16(r, 10u, b->log_hz);
    memcpy(&r[12], b->build, sizeof(b->build));
    seal(r);
}

void LogRec_PackAux(uint8_t r[LOGREC_SIZE], const LogAux *a)
{
    memset(r, 0, LOGREC_SIZE);
    r[0] = LOGREC_AUX; r[1] = a->phase; r[2] = a->nav_flags; r[3] = a->sats;
    put32(r, 4u, a->t_us);
    put16(r, 8u, a->seq);
    for (uint32_t i = 0u; i < 3u; i++) { put16(r, 10u + 2u*i, (uint16_t)q16(a->bias_dps[i], 100.0f)); }
    put16(r, 16u, (uint16_t)q16(a->gps_alt_m, 1.0f));
    r[18] = a->hdop10; r[19] = a->mode;
    put32(r, 20u, a->dropped);
    put16(r, 24u, a->wcet_us);
    put16(r, 26u, a->imu_fails);
    seal(r);
}

bool LogRec_Valid(const uint8_t r[LOGREC_SIZE])
{
    if ((r[0] != LOGREC_STATE) && (r[0] != LOGREC_EVENT) && (r[0] != LOGREC_BOOT) && (r[0] != LOGREC_AUX)) { return false; }
    return get16(r, LOGREC_SIZE - 2u) == crc16_ccitt(r, LOGREC_SIZE - 2u);
}

bool LogRec_UnpackState(const uint8_t r[LOGREC_SIZE], LogState *s)
{
    if ((r[0] != LOGREC_STATE) || !LogRec_Valid(r)) { return false; }
    s->phase = r[1]; s->mode = r[2]; s->flags = r[3];
    s->t_us = get32(r, 4u);
    s->seq  = get16(r, 8u);
    for (uint32_t i = 0u; i < 4u; i++) { s->q[i] = (float)(int16_t)get16(r, 10u + 2u*i) / 32767.0f; }
    for (uint32_t i = 0u; i < 3u; i++) {
        s->w_dps[i]    = (float)(int16_t)get16(r, 18u + 2u*i) / 10.0f;
        s->a_ms2[i]    = (float)(int16_t)get16(r, 24u + 2u*i) / 100.0f;
    }
    s->h_m      = (float)(int16_t)get16(r, 30u) / 10.0f;
    s->v_up     = (float)(int16_t)get16(r, 32u) / 100.0f;
    s->qbar_pa  = (float)get16(r, 34u) * 2.0f;
    s->trim_rad = (float)(int16_t)get16(r, 44u) / 1.0e4f;
    for (uint32_t i = 0u; i < 4u; i++) { s->fin[i] = (float)(int8_t)r[36u + i] / 127.0f; }
    s->loop_us = get16(r, 40u);
    s->faults  = get16(r, 42u);
    return true;
}
