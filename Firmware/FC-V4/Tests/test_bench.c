/* Bench profile player (BENCH-02): the firmware replay against the table that
 * Tools/bench/servo_id/profiles.py generated, and the timing a camera relies on. */
#include "unity.h"
#include "bench.h"
#include "alloc.h"
#include <math.h>
#include <string.h>

static Bench B;
void setUp(void) { Bench_Init(&B); }
void tearDown(void) {}

/* Play one whole run; return RUN frames seen. Checks PRE/POST and the LED. */
static uint32_t play(const BenchProfile *p, uint32_t *crc_out, int32_t *min_t, int32_t *max_t)
{
    TEST_ASSERT_TRUE(Bench_Start(&B, p, 0x04u));
    BenchOut o;
    for (uint32_t i = 0u; i < BENCH_PRE_FRAMES; i++) {
        TEST_ASSERT_TRUE(Bench_Active(&B));
        Bench_Step(&B, &o);
        TEST_ASSERT_FALSE(o.led);
        TEST_ASSERT_EQUAL_INT32(0, o.ticks);
    }
    uint32_t n = 0u;
    *min_t = 0; *max_t = 0;
    for (;;) {
        Bench_Step(&B, &o);
        TEST_ASSERT_EQUAL((((n / BENCH_LED_HALF) & 1u) == 0u), o.led);
        if (o.ticks < *min_t) { *min_t = o.ticks; }
        if (o.ticks > *max_t) { *max_t = o.ticks; }
        n++;
        if (o.done) { break; }
        TEST_ASSERT_TRUE(n < 100000u);
    }
    TEST_ASSERT_TRUE(o.led);                    /* the last RUN frame is lit: the end is an edge */
    *crc_out = B.crc;
    for (uint32_t i = 0u; i < BENCH_POST_FRAMES; i++) {
        TEST_ASSERT_TRUE(Bench_Active(&B));
        Bench_Step(&B, &o);
        TEST_ASSERT_FALSE(o.led);
        TEST_ASSERT_EQUAL_INT32(0, o.ticks);
    }
    TEST_ASSERT_FALSE(Bench_Active(&B));
    return n;
}

/* Every profile: the firmware plays exactly the sequence profiles.py computed
 * (same length, same CRC), within the command authority. */
static void test_every_profile_matches_the_generated_crc(void)
{
    TEST_ASSERT_TRUE(k_bench_nprofiles >= 10u);
    for (uint32_t i = 0u; i < k_bench_nprofiles; i++) {
        const BenchProfile *p = &k_bench_profiles[i];
        uint32_t crc = 0u; int32_t lo = 0, hi = 0;
        const uint32_t n = play(p, &crc, &lo, &hi);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(p->frames, n, p->name);
        TEST_ASSERT_EQUAL_HEX32_MESSAGE(p->crc, crc, p->name);
        TEST_ASSERT_TRUE(lo >= -BENCH_TICKS_MAX);
        TEST_ASSERT_TRUE(hi <= BENCH_TICKS_MAX);
    }
}

/* The LED count identifies the profile: length = 97 x a distinct odd number. */
static void test_lengths_identify_the_profile(void)
{
    for (uint32_t i = 0u; i < k_bench_nprofiles; i++) {
        const uint32_t n = k_bench_profiles[i].frames;
        TEST_ASSERT_EQUAL_UINT32(0u, n % BENCH_LED_HALF);
        TEST_ASSERT_EQUAL_UINT32(1u, (n / BENCH_LED_HALF) % 2u);
        for (uint32_t j = 0u; j < i; j++) { TEST_ASSERT_TRUE(k_bench_profiles[j].frames != n); }
    }
}

/* The constants the profiles were generated with are the firmware's. */
static void test_constants_match_the_flight_code(void)
{
    const float d2r = 0.017453292519943295f;
    const float dither_ticks = (CTRL_DITHER_AMP_RAD / (FIN_DEFLECT_DEG * d2r)) * ALLOC_TICKS_PER_NORM;
    TEST_ASSERT_EQUAL_INT32(BENCH_DITHER_TICKS, (int32_t)lroundf(dither_ticks));
    TEST_ASSERT_EQUAL_UINT32(CTRL_DITHER_PERIOD_N / 2u, BENCH_DITHER_HALF);
    TEST_ASSERT_EQUAL_INT32(BENCH_TICKS_MAX, Alloc_NormToTicks(1.0f));
    TEST_ASSERT_EQUAL_INT32(Alloc_NormToTicks(10.0f / FIN_DEFLECT_DEG), Bench_MdegToTicks(10000));
    TEST_ASSERT_EQUAL_INT32(-Alloc_NormToTicks(0.5f / FIN_DEFLECT_DEG), Bench_MdegToTicks(-500));
    TEST_ASSERT_EQUAL_INT32(BENCH_TICKS_MAX, Bench_MdegToTicks(99000));          /* clamped to 30 deg */
    TEST_ASSERT_EQUAL_UINT32(200u, FRAME_HZ);                                    /* profiles are in 200 Hz frames */
}

/* MOVE reaches its target on frame b and holds; SINE starts at the base, peaks
 * at +a a quarter period in; the dither is +,+,-,- and flips sign by fin. */
static void test_segment_arithmetic(void)
{
    static const BenchSeg seg[] = {
        { BSEG_MOVE, 0u, 10u, 100, 4u },                 /* ramp to 100 in 4 frames, hold 6 */
        { BSEG_MOVE, 0u, 3u, -50, 0u },                  /* step */
        { BSEG_MOVE, 0u, 1u, 0, 0u },
        { BSEG_SINE, BSEG_F_DITHER, 200u, 1000, 200u },  /* one cycle of 200 frames */
    };
    const BenchProfile p = { "t", seg, 4u, 214u, 0u };
    TEST_ASSERT_TRUE(Bench_Start(&B, &p, BENCH_MASK_ALL));
    BenchOut o;
    for (uint32_t i = 0u; i < BENCH_PRE_FRAMES; i++) { Bench_Step(&B, &o); }
    static const int32_t want[14] = { 25, 50, 75, 100, 100, 100, 100, 100, 100, 100, -50, -50, -50, 0 };
    for (uint32_t i = 0u; i < 14u; i++) {
        Bench_Step(&B, &o);
        TEST_ASSERT_EQUAL_INT32(want[i], o.ticks);
        TEST_ASSERT_EQUAL_INT32(0, o.dither);
    }
    for (uint32_t i = 0u; i < 200u; i++) {
        Bench_Step(&B, &o);
        const double ideal = 1000.0 * sin(6.283185307179586 * (double)i / 200.0);
        TEST_ASSERT_TRUE(fabs((double)o.ticks - ideal) < 7.0);          /* 1024-step table: < 0.7 % */
        if (i == 0u)  { TEST_ASSERT_EQUAL_INT32(0, o.ticks); }
        if (i == 50u) { TEST_ASSERT_INT32_WITHIN(1, 1000, o.ticks); }
        const uint32_t k = 14u + i;                                     /* dither phase runs from RUN frame 0 */
        TEST_ASSERT_EQUAL_INT32((((k / 2u) & 1u) == 0u) ? BENCH_DITHER_TICKS : -BENCH_DITHER_TICKS, o.dither);
        TEST_ASSERT_EQUAL_INT32(o.ticks + o.dither, Bench_FinTicks(&o, 0u));
        TEST_ASSERT_EQUAL_INT32(o.ticks - o.dither, Bench_FinTicks(&o, 1u));
        TEST_ASSERT_EQUAL_INT32(o.ticks + o.dither, Bench_FinTicks(&o, 2u));
        TEST_ASSERT_EQUAL_INT32(o.ticks - o.dither, Bench_FinTicks(&o, 3u));
        TEST_ASSERT_EQUAL(i == 199u, o.done);
    }
}

/* Only the selected fins move; hold keeps one angle until stopped; stop is immediate. */
static void test_mask_hold_and_stop(void)
{
    BenchOut o;
    Bench_Hold(&B, 0x04u, Bench_MdegToTicks(10000));
    for (uint32_t i = 0u; i < 1000u; i++) { Bench_Step(&B, &o); }
    TEST_ASSERT_TRUE(Bench_Active(&B));
    TEST_ASSERT_TRUE(o.led);
    TEST_ASSERT_EQUAL_INT32(828, Bench_FinTicks(&o, 2u));               /* 10 deg x 82.76 */
    TEST_ASSERT_EQUAL_INT32(0, Bench_FinTicks(&o, 0u));
    TEST_ASSERT_EQUAL_INT32(0, Bench_FinTicks(&o, 3u));
    TEST_ASSERT_EQUAL_INT32(0, Bench_FinTicks(&o, 7u));
    Bench_Hold(&B, BENCH_MASK_ALL, 999999);
    Bench_Step(&B, &o);
    TEST_ASSERT_EQUAL_INT32(BENCH_TICKS_MAX, Bench_FinTicks(&o, 1u));   /* clamped */
    Bench_Stop(&B);
    TEST_ASSERT_FALSE(Bench_Active(&B));
    TEST_ASSERT_FALSE(Bench_Start(&B, NULL, BENCH_MASK_ALL));
    TEST_ASSERT_FALSE(Bench_Start(&B, &k_bench_profiles[0], 0u));
    TEST_ASSERT_FALSE(Bench_Active(&B));
}

static void test_find_by_name_or_id(void)
{
    TEST_ASSERT_EQUAL_PTR(&k_bench_profiles[0], Bench_Find("sync"));
    TEST_ASSERT_EQUAL_PTR(&k_bench_profiles[1], Bench_Find("stair"));
    TEST_ASSERT_EQUAL_PTR(&k_bench_profiles[1], Bench_Find("1"));
    TEST_ASSERT_NULL(Bench_Find("999"));
    TEST_ASSERT_NULL(Bench_Find("nope"));
    TEST_ASSERT_NULL(Bench_Find(""));
    TEST_ASSERT_NULL(Bench_Find(NULL));
    TEST_ASSERT_NULL(Bench_Find("1x"));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_every_profile_matches_the_generated_crc);
    RUN_TEST(test_lengths_identify_the_profile);
    RUN_TEST(test_constants_match_the_flight_code);
    RUN_TEST(test_segment_arithmetic);
    RUN_TEST(test_mask_hold_and_stop);
    RUN_TEST(test_find_by_name_or_id);
    return UNITY_END();
}
