#include "unity.h"
#include "detect.h"

static Detect D;
#define DT 0.01f
#define G 9.80665f

static DetectCfg cfg(void)
{
    DetectCfg c = { .liftoff_acc = 3.0f * G, .liftoff_n = 10u, .liftoff_v = 8.0f, .liftoff_n_nov = 30u, .t_boost_min = 0.1f,
                    .burnout_acc = 0.0f, .burnout_n = 5u, .apogee_n = 5u, .t_apogee_max = 15.0f,
                    .t_land_min = 5.0f, .t_land_max = 300.0f, .land_acc_tol = 1.0f,
                    .land_gyro = 0.2f, .land_v = 1.5f, .land_n = 200u };
    return c;
}

void setUp(void) { DetectCfg c = cfg(); Detect_Init(&D, &c); }
void tearDown(void) {}

static Phase run(Phase ph, DetectIn in, uint32_t n)
{
    for (uint32_t i = 0u; i < n; i++) {
        const Phase to = Detect_Step(&D, ph, &in, DT);
        if (to != ph) { return to; }
    }
    return ph;
}

static void test_liftoff_needs_persistence(void)
{
    DetectIn knock = { .f_axial = 6.0f * G, .f_norm = 6.0f * G, .v_up = 2.0f, .v_valid = true };
    TEST_ASSERT_EQUAL_HEX16(PH_READY, run(PH_READY, knock, 25u));   /* long knock, little velocity */
    DetectIn rest = { .f_axial = G, .f_norm = G };
    TEST_ASSERT_EQUAL_HEX16(PH_READY, run(PH_READY, rest, 1u));
    DetectIn motor = { .f_axial = 8.0f * G, .f_norm = 8.0f * G, .v_up = 9.0f, .v_valid = true };
    TEST_ASSERT_EQUAL_HEX16(PH_READY, run(PH_READY, motor, 9u));
    TEST_ASSERT_EQUAL_HEX16(PH_BOOST, run(PH_READY, motor, 1u));    /* 100 ms + 8 m/s */
}

/* Nav invalid (saturated): acceleration alone, but for 300 ms. */
static void test_liftoff_without_nav(void)
{
    DetectIn sat = { .f_axial = 15.0f * G, .f_norm = 15.0f * G, .v_valid = false };
    TEST_ASSERT_FALSE(Detect_LiftoffPending(&D));
    TEST_ASSERT_EQUAL_HEX16(PH_READY, run(PH_READY, sat, 29u));
    TEST_ASSERT_TRUE(Detect_LiftoffPending(&D));          /* arbiter must hold READY */
    TEST_ASSERT_EQUAL_HEX16(PH_BOOST, run(PH_READY, sat, 1u));
}

static void test_no_liftoff_outside_ready(void)
{
    DetectIn boost = { .f_axial = 10.0f * G };
    TEST_ASSERT_EQUAL_HEX16(PH_SETTLING, run(PH_SETTLING, boost, 100u));
}

static void test_burnout_after_min_time(void)
{
    DetectIn coast = { .f_axial = -2.0f };
    /* negative axial f from t=0 is ignored until t_boost_min */
    TEST_ASSERT_EQUAL_HEX16(PH_COAST, run(PH_BOOST, coast, 20u));
    TEST_ASSERT_TRUE(D.t >= 0.1f);
}

static void test_apogee_on_velocity_or_timer(void)
{
    DetectIn up = { .v_up = 50.0f, .v_valid = true };
    TEST_ASSERT_EQUAL_HEX16(PH_COAST, run(PH_COAST, up, 100u));
    DetectIn down = { .v_up = -0.5f, .v_valid = true };
    TEST_ASSERT_EQUAL_HEX16(PH_DESCENT, run(PH_COAST, down, 5u));

    setUp();
    DetectIn bad = { .v_up = -10.0f, .v_valid = false };       /* saturated nav: timer only */
    TEST_ASSERT_EQUAL_HEX16(PH_COAST, run(PH_COAST, bad, 1400u));
    TEST_ASSERT_EQUAL_HEX16(PH_DESCENT, run(PH_COAST, bad, 200u));
}

static void test_landing_needs_gps_or_backup(void)
{
    DetectIn chute = { .f_norm = G, .w_norm = 0.0f, .v_up = -5.0f, .v_gps = true };
    TEST_ASSERT_EQUAL_HEX16(PH_DESCENT, run(PH_DESCENT, chute, 2000u));
    DetectIn ground = { .f_norm = G, .w_norm = 0.0f, .v_up = 0.1f, .v_gps = true };
    TEST_ASSERT_EQUAL_HEX16(PH_LANDED, run(PH_DESCENT, ground, 200u));

    setUp();
    DetectIn nogps = { .f_norm = G, .v_up = 0.0f, .v_gps = false };
    TEST_ASSERT_EQUAL_HEX16(PH_DESCENT, run(PH_DESCENT, nogps, 29000u));
    TEST_ASSERT_EQUAL_HEX16(PH_LANDED, run(PH_DESCENT, nogps, 1001u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_liftoff_needs_persistence);
    RUN_TEST(test_liftoff_without_nav);
    RUN_TEST(test_no_liftoff_outside_ready);
    RUN_TEST(test_burnout_after_min_time);
    RUN_TEST(test_apogee_on_velocity_or_timer);
    RUN_TEST(test_landing_needs_gps_or_backup);
    return UNITY_END();
}
