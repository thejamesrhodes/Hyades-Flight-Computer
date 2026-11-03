/* ============================================================================
 * fdir.c  —  Consistency engine (see fdir.h). Thresholds: board.h section 8.
 * ============================================================================ */
#include "fdir.h"
#include "board.h"

/* Instance table (FDS-01 §4.2 subset implemented on FC-V4). Bounds are named
 * constants in board.h with their derivation; nothing here is tuned ad hoc. */
static const MonCfg k_mon[MON_COUNT] = {
    [MON_IMU_READ]    = { "imu_read",  0.0f, 0.5f,                FDIR_IMU_READ_N,  SEV_HARD, SEV_HARD, EFF_NO_IMU },
    [MON_ACCEL_RANGE] = { "accel_rng", 0.0f, FDIR_ACCEL_MAX_MS2,  FDIR_RANGE_N,     SEV_SOFT, SEV_SOFT, EFF_NONE },
    [MON_GYRO_RANGE]  = { "gyro_rng",  0.0f, FDIR_GYRO_MAX_DPS,   FDIR_RANGE_N,     SEV_SOFT, SEV_SOFT, EFF_NO_ESTIMATOR },
    [MON_LOOP_TIME]   = { "loop_time", 0.0f, FDIR_LOOP_BUDGET_US, FDIR_LOOP_N,      SEV_SOFT, SEV_SOFT, EFF_NONE },
    [MON_OVERRUN]     = { "overrun",   0.0f, 0.5f,                FDIR_OVERRUN_N,   SEV_HARD, SEV_SOFT, EFF_NONE },
    [MON_EST_FINITE]  = { "est_nan",   0.0f, 0.5f,                1u,               SEV_HARD, SEV_SOFT, EFF_NO_ESTIMATOR },
    [MON_EST_COV]     = { "est_tilt",  0.0f, FDIR_EST_TILT_VAR_MAX, FDIR_EST_COV_N, SEV_SOFT, SEV_SOFT, EFF_NO_ESTIMATOR },
    [MON_GPS_LINK]    = { "gps_link",  0.0f, 0.5f,                1u,               SEV_INFO, SEV_INFO, EFF_NONE },
};

void Fdir_Init(Fdir *f)
{
    for (uint32_t i = 0u; i < (uint32_t)MON_COUNT; i++) {
        f->count[i] = 0u; f->trips[i] = 0u; f->last_r[i] = 0.0f;
    }
    f->active = 0u;
    f->latch = 0u; f->latch_inv = ~0u;
}

const MonCfg *Fdir_Config(MonId id) { return &k_mon[id]; }

bool Fdir_LatchRead(const Fdir *f, uint32_t *latch)
{
    if ((f->latch ^ f->latch_inv) != 0xFFFFFFFFu) { return false; }
    *latch = f->latch;
    return true;
}

bool Fdir_Eval(Fdir *f, MonId id, float r, bool in_flight)
{
    const MonCfg *c = &k_mon[id];
    const uint32_t bit = 1u << (uint32_t)id;
    f->last_r[id] = r;

    /* NaN fails the band test by construction (all comparisons false). */
    const bool in_band = (r >= c->lo) && (r <= c->hi);
    if (in_band) {
        f->count[id] = 0u;
        f->active &= ~bit;
        return false;
    }
    if (f->count[id] < 0xFFFFu) { f->count[id]++; }
    if (f->count[id] < c->n_persist) { return false; }

    if ((f->active & bit) == 0u) {
        f->active |= bit;
        if (f->trips[id] < 0xFFFFu) { f->trips[id]++; }
    }
    if ((in_flight ? c->sev_flight : c->sev_pad) == (uint8_t)SEV_HARD) {
        uint32_t l;
        if (!Fdir_LatchRead(f, &l)) { l = 0xFFFFFFFFu; }   /* corrupt: latch all */
        l |= bit;
        f->latch = l; f->latch_inv = ~l;
    }
    return true;
}

uint32_t Fdir_HardMask(bool in_flight)
{
    uint32_t m = 0u;
    for (uint32_t i = 0u; i < (uint32_t)MON_COUNT; i++) {
        if ((in_flight ? k_mon[i].sev_flight : k_mon[i].sev_pad) == (uint8_t)SEV_HARD) { m |= 1u << i; }
    }
    return m;
}

void Fdir_ClearLatch(Fdir *f)
{
    for (uint32_t i = 0u; i < (uint32_t)MON_COUNT; i++) { f->count[i] = 0u; }
    f->active = 0u;                           /* monitors re-trip on the next frame if still failing */
    f->latch = 0u; f->latch_inv = ~0u;
}

FdirSev Fdir_Worst(const Fdir *f, bool in_flight)
{
    uint32_t l;
    if (!Fdir_LatchRead(f, &l) || (l != 0u)) { return SEV_HARD; }
    FdirSev worst = SEV_INFO;
    for (uint32_t i = 0u; i < (uint32_t)MON_COUNT; i++) {
        if ((f->active & (1u << i)) == 0u) { continue; }
        const FdirSev s = (FdirSev)(in_flight ? k_mon[i].sev_flight : k_mon[i].sev_pad);
        if (s > worst) { worst = s; }
    }
    return worst;
}

FdirCap Fdir_Capability(const Fdir *f)
{
    FdirCap cap = CAP_FULL;
    uint32_t l;
    const uint32_t bits = f->active | (Fdir_LatchRead(f, &l) ? l : 0xFFFFFFFFu);
    for (uint32_t i = 0u; i < (uint32_t)MON_COUNT; i++) {
        if ((bits & (1u << i)) == 0u) { continue; }
        if (k_mon[i].effect == (uint8_t)EFF_NO_IMU)            { return CAP_NONE; }
        if (k_mon[i].effect == (uint8_t)EFF_NO_ESTIMATOR)       { cap = CAP_GYRO_ONLY; }
    }
    return cap;
}
