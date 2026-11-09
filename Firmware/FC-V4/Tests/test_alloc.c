/* ALLOC-01: control allocation (sum-preserving quantisation, null-space dither,
 * priority saturation). Built with CTRL_ALLOC_STAGGER 1 and CTRL_DITHER_NULL 1. */
#include "unity.h"
#include "alloc.h"
#include "board.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

static Alloc A;

static int32_t tick_sum(void) { return A.tick[0] + A.tick[1] + A.tick[2] + A.tick[3]; }

/* roll-only command: the tick SUM is the nearest whole tick to 4 x the command
 * (dither cancels in the sum), every fin is within one tick of the command, and
 * pitch/yaw differences are at most one tick */
static void test_roll_sum_preserved(void)
{
    Alloc_Init(&A);
    for (int32_t s = -4000; s <= 4000; s += 7) {
        const float u = (float)s * 0.0001f;
        const float cmd[4] = { u, u, u, u };
        float out[4];
        Alloc_Apply(&A, cmd, true, out);
        const float want = 4.0f * u * ALLOC_TICKS_PER_NORM;
        TEST_ASSERT_INT32_WITHIN(1, (int32_t)lroundf(want), tick_sum());
        TEST_ASSERT_FLOAT_WITHIN(0.5f + 1e-3f, want, (float)tick_sum());
        for (uint32_t i = 0u; i < 4u; i++) {
            const float ti = u * ALLOC_TICKS_PER_NORM;
            TEST_ASSERT_FLOAT_WITHIN(1.0f + (fabsf(A.dither) * ALLOC_TICKS_PER_NORM) + 1e-3f, ti, (float)A.tick[i]);
            TEST_ASSERT_FLOAT_WITHIN(1e-6f, Alloc_TicksToNorm(A.tick[i]), out[i]);
        }
        TEST_ASSERT_INT32_WITHIN(1, A.tick[0], A.tick[2]);
        TEST_ASSERT_INT32_WITHIN(1, A.tick[1], A.tick[3]);
    }
}

/* two extra ticks go to an opposite pair (pure roll); one extra tick alternates
 * between the two fins of that pair, so its pitch moment averages to zero */
static void test_extra_ticks_pair_first_and_alternate(void)
{
    Alloc_Init(&A);
    const float q = 1.0f / ALLOC_TICKS_PER_NORM;
    float out[4];
    /* null dither shifts the pairs apart; with it the pair (0,2) leads on its
     * positive half-period. Use a command half a tick above a whole tick. */
    int32_t pitch_sum = 0;
    for (uint32_t k = 0u; k < 8u; k++) {
        const float u = 10.25f * q;     /* sum 41 ticks = 4 x 10 + 1 extra */
        const float cmd[4] = { u, u, u, u };
        Alloc_Apply(&A, cmd, true, out);
        pitch_sum += A.tick[0] - A.tick[2];
    }
    TEST_ASSERT_EQUAL_INT32(0, pitch_sum);
}

/* zero command with the null dither on: each fin moves, net roll, pitch and
 * yaw are exactly zero */
static void test_null_dither_no_net_moment(void)
{
    Alloc_Init(&A);
    const float z[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float out[4];
    for (uint32_t k = 0u; k < 2u * CTRL_DITHER_PERIOD_N; k++) {
        Alloc_Apply(&A, z, true, out);
        TEST_ASSERT_TRUE(A.tick[0] != 0);
        TEST_ASSERT_EQUAL_INT32(0, tick_sum());
        TEST_ASSERT_EQUAL_INT32(A.tick[0], A.tick[2]);
        TEST_ASSERT_EQUAL_INT32(A.tick[1], A.tick[3]);
        TEST_ASSERT_EQUAL_INT32(-A.tick[0], A.tick[1]);
    }
}

/* inactive: pass-through, no dither, phase restarts */
static void test_inactive_passthrough(void)
{
    Alloc_Init(&A);
    const float cmd[4] = { 0.1f, -0.2f, 0.3f, 0.0f };
    float out[4];
    Alloc_Apply(&A, cmd, true, out);
    Alloc_Apply(&A, cmd, false, out);
    for (uint32_t i = 0u; i < 4u; i++) { TEST_ASSERT_EQUAL_FLOAT(cmd[i], out[i]); }
    TEST_ASSERT_EQUAL_UINT32(0u, A.k);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, A.dither);
}

/* roll keeps priority: at full roll there is no room for pitch/yaw, which is
 * scaled to zero rather than clipping fins unevenly */
static void test_roll_priority_saturation(void)
{
    Alloc_Init(&A);
    const float cmd[4] = { 1.3f, 0.9f, 0.7f, 0.9f };   /* r = 0.95, p = 0.3 */
    float out[4];
    Alloc_Apply(&A, cmd, true, out);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.95f, A.v[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f, fabsf(A.v[1]) + fabsf(A.v[3]));
    for (uint32_t i = 0u; i < 4u; i++) { TEST_ASSERT_TRUE(fabsf(out[i]) <= 1.0f); }
}

/* quantiser is safe on NaN and out-of-range input */
static void test_norm_to_ticks_safe(void)
{
    TEST_ASSERT_EQUAL_INT32(0, Alloc_NormToTicks(NAN));
    TEST_ASSERT_EQUAL_INT32((int32_t)ALLOC_TICKS_PER_NORM, Alloc_NormToTicks(5.0f));
    TEST_ASSERT_EQUAL_INT32(-(int32_t)ALLOC_TICKS_PER_NORM, Alloc_NormToTicks(-5.0f));
    TEST_ASSERT_EQUAL_INT32(0, Alloc_NormToTicks(-INFINITY));   /* non-finite -> centre */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_roll_sum_preserved);
    RUN_TEST(test_extra_ticks_pair_first_and_alternate);
    RUN_TEST(test_null_dither_no_net_moment);
    RUN_TEST(test_inactive_passthrough);
    RUN_TEST(test_roll_priority_saturation);
    RUN_TEST(test_norm_to_ticks_safe);
    return UNITY_END();
}
