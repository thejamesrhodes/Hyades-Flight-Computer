#include "unity.h"
#include "nav.h"
#include <math.h>

static Nav N;
static const float Q_UP[4] = { 1.0f, 0.0f, 0.0f, 0.0f };   /* body Z = up (nose up) */
static const float W0[3]   = { 0.0f, 0.0f, 0.0f };
#define DT 0.01f

static NavCfg cfg(void)
{
    NavCfg c = { .h_site_m = 100.0f, .sat_mps2 = 15.5f * 9.80665f, .still_acc_mps2 = 0.5f,
                 .still_gyro_radps = 0.1f, .acc_psd = 0.01f, .gps_var_m2 = 25.0f,
                 .gps_nis_max = 9.0f, .gps_rej_reset = 3u, .gps_v_var = 100.0f,
                 .q_speed_min = 1.0f };
    return c;
}

void setUp(void) { NavCfg c = cfg(); Nav_Init(&N, &c); }
void tearDown(void) {}

static void pad(float g, uint32_t n)
{
    const float f[3] = { 0.0f, 0.0f, g };
    for (uint32_t i = 0u; i < n; i++) { Nav_Step(&N, Q_UP, f, W0, DT, true); }
}

static void test_pad_measures_gravity_and_holds_zero(void)
{
    pad(9.79f, 200u);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 9.79f, N.g_ref);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, Nav_Height(&N));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, Nav_VUp(&N));
    TEST_ASSERT_TRUE((N.flags & NAV_F_INIT) != 0u);
    TEST_ASSERT_TRUE(Nav_QValid(&N));
}

/* Constant 50 m/s^2 net up for 2 s: v = 100 m/s, h = 100 m (trapezoid exact). */
static void test_vertical_boost_integrates_exactly(void)
{
    pad(9.81f, 100u);
    const float f[3] = { 0.0f, 0.0f, 9.81f + 50.0f };
    for (uint32_t i = 0u; i < 200u; i++) { Nav_Step(&N, Q_UP, f, W0, DT, false); }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, Nav_VUp(&N));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 100.0f, Nav_Height(&N));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, N.speed);
}

/* Liftoff while still flagged on_pad: the first high-g sample is not stationary,
 * so it is integrated, not zeroed (detector latency must not lose velocity). */
static void test_liftoff_integrated_from_first_sample(void)
{
    pad(9.81f, 100u);
    const float f[3] = { 0.0f, 0.0f, 9.81f + 100.0f };
    for (uint32_t i = 0u; i < 5u; i++) { Nav_Step(&N, Q_UP, f, W0, DT, true); }  /* detector window */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.0f, Nav_VUp(&N));
}

/* Nose tilted 90 deg about +X: body Z maps to world -Y (R = Rx(90)). */
static void test_attitude_rotates_specific_force(void)
{
    pad(9.81f, 100u);
    const float s = sqrtf(0.5f);
    const float q[4] = { s, s, 0.0f, 0.0f };
    const float f[3] = { 0.0f, 9.81f, 10.0f };            /* gravity reaction + 10 m/s^2 thrust */
    for (uint32_t i = 0u; i < 100u; i++) { Nav_Step(&N, q, f, W0, DT, false); }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.0f, N.v[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, N.v[2]);
}

static void test_saturation_flags_until_pad(void)
{
    pad(9.81f, 100u);
    const float f[3] = { 0.0f, 0.0f, 16.0f * 9.80665f };
    Nav_Step(&N, Q_UP, f, W0, DT, false);
    TEST_ASSERT_FALSE(Nav_QValid(&N));
    pad(9.81f, 1u);
    TEST_ASSERT_TRUE(Nav_QValid(&N));
}

static void test_isa_density_and_qbar(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.2250f, Nav_IsaDensity(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(2e-3f, 1.1117f, Nav_IsaDensity(1000.0f));
    pad(9.81f, 10u);
    N.v[2] = 100.0f;
    const float f[3] = { 0.0f, 0.0f, 9.81f };             /* coast at constant v */
    Nav_Step(&N, Q_UP, f, W0, DT, false);
    const float rho = Nav_IsaDensity(100.0f + Nav_Height(&N));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.5f * rho * 1e4f, N.qbar);
}

static void test_gps_pulls_height_and_gates_outliers(void)
{
    pad(9.81f, 10u);
    N.p[2] = 50.0f; N.P[0] = 400.0f; N.P[3] = 4.0f;       /* drifted, uncertain */
    TEST_ASSERT_TRUE(Nav_GpsAlt(&N, 100.0f + 30.0f));     /* truth h = 30 */
    TEST_ASSERT_TRUE(Nav_Height(&N) < 34.0f);
    TEST_ASSERT_TRUE(Nav_Height(&N) > 30.0f);
    TEST_ASSERT_FALSE(Nav_GpsAlt(&N, 100.0f + 900.0f));   /* NIS gate */
    TEST_ASSERT_FALSE(Nav_GpsAlt(&N, NAN));
}

/* A drifted solution must not lock GPS out: 3 rejections in a row re-anchor. */
static void test_gps_reanchors_after_repeated_rejection(void)
{
    pad(9.81f, 10u);
    N.p[2] = -5000.0f; N.P[0] = 1.0f;                     /* confidently wrong */
    TEST_ASSERT_FALSE(Nav_GpsAlt(&N, 100.0f + 300.0f));
    TEST_ASSERT_FALSE(Nav_GpsAlt(&N, 100.0f + 300.0f));
    TEST_ASSERT_TRUE(Nav_GpsAlt(&N, 100.0f + 300.0f));    /* third: re-anchor */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 300.0f, Nav_Height(&N));
    TEST_ASSERT_TRUE((N.flags & NAV_F_GPS_AIDED) != 0u);
}

/* Saturated (velocity known bad): the first fix re-anchors immediately. */
static void test_gps_reanchors_at_once_when_saturated(void)
{
    pad(9.81f, 10u);
    const float f[3] = { 0.0f, 0.0f, 20.0f * 9.80665f };
    Nav_Step(&N, Q_UP, f, W0, DT, false);
    N.p[2] = 4000.0f;
    TEST_ASSERT_TRUE(Nav_GpsAlt(&N, 100.0f + 250.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 250.0f, Nav_Height(&N));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, Nav_VUp(&N));
}

static void test_pad_gps_sets_site(void)
{
    Nav_PadGps(&N, 120.0f); Nav_PadGps(&N, 124.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 122.0f, N.h_site);
    TEST_ASSERT_TRUE((N.flags & NAV_F_GPS_SITE) != 0u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_pad_measures_gravity_and_holds_zero);
    RUN_TEST(test_vertical_boost_integrates_exactly);
    RUN_TEST(test_liftoff_integrated_from_first_sample);
    RUN_TEST(test_attitude_rotates_specific_force);
    RUN_TEST(test_saturation_flags_until_pad);
    RUN_TEST(test_isa_density_and_qbar);
    RUN_TEST(test_gps_pulls_height_and_gates_outliers);
    RUN_TEST(test_pad_gps_sets_site);
    RUN_TEST(test_gps_reanchors_after_repeated_rejection);
    RUN_TEST(test_gps_reanchors_at_once_when_saturated);
    return UNITY_END();
}
