/* End-to-end tests of the portable flight pipeline (flight.c) with synthetic,
 * vertically flown IMU data at the firmware frame rate. Pins the phase
 * sequence and the inertial landing rules (ADR-004 §5). */
#include "unity.h"
#include "flight.h"
#include "board.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

void setUp(void) {}
void tearDown(void) {}

static Flight F;
static uint32_t n_phase;
static Phase seq[16];
static void on_ev(void *ctx, EventCode c, uint32_t a, const char *t)
{
    (void)ctx; (void)t;
    if ((c == EV_PHASE) && (n_phase < 16u)) { seq[n_phase++] = (Phase)a; }
}

static const NavCfg NC = {
    .h_site_m = NAV_SITE_ELEV_M, .sat_mps2 = NAV_SAT_MPS2, .sat_budget_n = NAV_SAT_BUDGET_N, .still_acc_mps2 = NAV_STILL_ACC_MPS2,
    .still_gyro_radps = NAV_STILL_GYRO_RADPS, .acc_psd = NAV_ACC_PSD, .gps_var_m2 = NAV_GPS_VAR_M2,
    .gps_nis_max = NAV_GPS_NIS_MAX, .gps_rej_reset = NAV_GPS_REJ_RESET, .gps_v_var = NAV_GPS_V_VAR,
    .q_speed_min = NAV_Q_SPEED_MIN,
};
static const DetectCfg DC = {
    .liftoff_acc = DET_LIFTOFF_ACC, .liftoff_n = DET_LIFTOFF_N, .liftoff_v = DET_LIFTOFF_V_MPS,
    .liftoff_n_nov = DET_LIFTOFF_N_NOV, .t_boost_min = DET_T_BOOST_MIN_S, .burnout_acc = DET_BURNOUT_ACC,
    .burnout_n = DET_BURNOUT_N, .apogee_n = DET_APOGEE_N, .t_apogee_max = DET_T_APOGEE_MAX_S,
    .t_land_min = DET_T_LAND_MIN_S, .t_land_max = DET_T_LAND_MAX_S, .land_acc_tol = DET_LAND_ACC_TOL,
    .land_gyro = DET_LAND_GYRO_RADPS, .land_v = DET_LAND_V_MPS, .land_n = DET_LAND_N,
    .land_tau = DET_LAND_TAU_S, .land_still_sd = DET_LAND_STILL_SD, .land_still_gyro = DET_LAND_STILL_GYRO,
    .land_shock_acc = DET_LAND_SHOCK_ACC, .land_n_shock = DET_LAND_N_SHOCK, .land_n_still = DET_LAND_N_STILL,
};

static uint32_t rng = 12345u;
static float nrand(void)                   /* crude N(0,1): sum of 12 uniforms */
{
    float s = 0.0f;
    for (int i = 0; i < 12; i++) { rng = rng * 1664525u + 1013904223u; s += (float)(rng >> 8) / 16777216.0f; }
    return s - 6.0f;
}

static uint32_t now_ms;
static void step(float fz, float fx, float wx_dps)
{
    FlightIn in;
    memset(&in, 0, sizeof in);
    in.imu_ok = true;
    in.accel[0] = fx + (0.006f * nrand()); in.accel[1] = 0.006f * nrand(); in.accel[2] = fz + (0.006f * nrand());
    in.gyro_dps[0] = wx_dps; in.gyro_dps[1] = 0.0f; in.gyro_dps[2] = 0.0f;
    in.loop_us = 800u; in.now_ms = now_ms;
    Flight_Step(&F, &in, FRAME_DT_S);
    now_ms += 1000u / FRAME_HZ;
}

static void boot_to_ready(void)
{
    Flight_Init(&F, &NC, &DC, MS_TO_FRAMES(READY_HOLD_MS), NAV_BRIDGE_N, false, on_ev, NULL);
    n_phase = 0u; now_ms = 0u;
    const float a0[3] = { 0.0f, 0.0f, BOARD_G_MPS2 }, m0[3] = { 0.0f, 0.0f, 0.0f };
    IEKF_Seed(&F.iekf, a0, m0);
    Flight_GoPhase(&F, PH_CALIBRATE);
    Flight_GoPhase(&F, PH_SETTLING);
    for (uint32_t i = 0u; i < 3u * FRAME_HZ; i++) { step(BOARD_G_MPS2, 0.0f, 0.0f); }
    if (Flight_Phase(&F) != PH_READY) {
        for (uint32_t i = 0u; i < n_phase; i++) { printf("seq %s\n", Phase_Name(seq[i])); }
        printf("active %lx latch %lx\n", (unsigned long)F.fdir.active, (unsigned long)F.fdir.latch);
    }
    TEST_ASSERT_EQUAL(PH_READY, Flight_Phase(&F));
}

/* boost 8 g for 1.5 s, coast to apogee, then descent under a canopy */
static void fly_to_descent(void)
{
    for (uint32_t i = 0u; i < (3u * FRAME_HZ) / 2u; i++) { step(8.0f * BOARD_G_MPS2, 0.0f, 0.0f); }
    TEST_ASSERT_EQUAL(PH_BOOST, seq[3]);
    /* coast: drag decelerates (f_axial < 0) until v_up < 0 */
    for (uint32_t i = 0u; (i < 15u * FRAME_HZ) && (Flight_Phase(&F) != PH_DESCENT); i++) { step(-2.0f, 0.0f, 0.0f); }
    TEST_ASSERT_EQUAL(PH_DESCENT, Flight_Phase(&F));
}

/* canopy: |f| ~ g with turbulence 0.3 m/s^2 rms and a 0.4 Hz swing of 0.4 rad/s */
static void canopy(float seconds)
{
    const uint32_t n = (uint32_t)(seconds * (float)FRAME_HZ);
    for (uint32_t i = 0u; i < n; i++) {
        const float ph = 6.2831853f * 0.4f * (float)i / (float)FRAME_HZ;
        step(BOARD_G_MPS2 + (0.3f * nrand()), 0.5f * sinf(ph), 23.0f * cosf(ph));
    }
}

static void test_phase_sequence_nominal(void)
{
    boot_to_ready();
    fly_to_descent();
    TEST_ASSERT_EQUAL(PH_COAST, seq[4]);
    TEST_ASSERT_EQUAL(PH_DESCENT, seq[5]);
}

static void test_no_false_landing_under_canopy(void)
{
    boot_to_ready(); fly_to_descent();
    canopy(60.0f);
    TEST_ASSERT_EQUAL(PH_DESCENT, Flight_Phase(&F));
}

static void test_landing_after_shock_within_3s(void)
{
    boot_to_ready(); fly_to_descent();
    canopy(20.0f);
    for (uint32_t i = 0u; i < MS_TO_FRAMES(60u); i++) { step(9.0f * BOARD_G_MPS2, 0.0f, 0.0f); }   /* impact */
    uint32_t k = 0u;
    for (; (k < 15u * FRAME_HZ) && (Flight_Phase(&F) != PH_LANDED); k++) { step(BOARD_G_MPS2, 0.0f, 0.0f); }
    TEST_ASSERT_EQUAL(PH_LANDED, Flight_Phase(&F));
    TEST_ASSERT_LESS_THAN_UINT32(3u * FRAME_HZ, k);     /* 2 s window + EWMA settling */
}

static void test_landing_without_shock_or_gps_within_32s(void)
{
    boot_to_ready(); fly_to_descent();
    canopy(20.0f);
    uint32_t k = 0u;
    for (; (k < 40u * FRAME_HZ) && (Flight_Phase(&F) != PH_LANDED); k++) { step(BOARD_G_MPS2, 0.0f, 0.0f); }
    TEST_ASSERT_EQUAL(PH_LANDED, Flight_Phase(&F));
    TEST_ASSERT_GREATER_THAN_UINT32(30u * FRAME_HZ, k);
    TEST_ASSERT_LESS_THAN_UINT32(32u * FRAME_HZ, k);
}

static void test_shadow_mode_holds_canards(void)
{
    boot_to_ready();
    F.shadow = true;
    for (uint32_t i = 0u; i < FRAME_HZ; i++) { step(8.0f * BOARD_G_MPS2, 0.0f, 0.0f); }
    FlightIn in; memset(&in, 0, sizeof in);
    in.imu_ok = true; in.accel[2] = 8.0f * BOARD_G_MPS2; in.gyro_dps[2] = 200.0f; in.now_ms = now_ms;
    Flight_Step(&F, &in, FRAME_DT_S);
    TEST_ASSERT_EQUAL(PH_BOOST, Flight_Phase(&F));
    TEST_ASSERT_TRUE(fabsf(F.fin_cmd[0]) > 0.001f);     /* law ran */
    for (int i = 0; i < 4; i++) { TEST_ASSERT_EQUAL_FLOAT(0.0f, F.fin[i]); }
}

static void test_corrupt_word_in_flight_goes_cato_then_lands(void)
{
    boot_to_ready(); fly_to_descent();
    F.ph.inv = 0u;                                   /* corrupt the protected word */
    step(BOARD_G_MPS2, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL(PH_CATO_SAFE, Flight_Phase(&F));
    canopy(20.0f);
    for (uint32_t i = 0u; i < MS_TO_FRAMES(60u); i++) { step(9.0f * BOARD_G_MPS2, 0.0f, 0.0f); }
    for (uint32_t k = 0u; (k < 15u * FRAME_HZ) && (Flight_Phase(&F) != PH_LANDED); k++) { step(BOARD_G_MPS2, 0.0f, 0.0f); }
    TEST_ASSERT_EQUAL(PH_LANDED, Flight_Phase(&F));
}

static void test_liftoff_from_settling_is_sequenced(void)
{
    Flight_Init(&F, &NC, &DC, MS_TO_FRAMES(60000u), NAV_BRIDGE_N, false, on_ev, NULL);   /* READY never reached */
    n_phase = 0u; now_ms = 0u;
    const float a0[3] = { 0.0f, 0.0f, BOARD_G_MPS2 }, m0[3] = { 0.0f, 0.0f, 0.0f };
    IEKF_Seed(&F.iekf, a0, m0);
    Flight_GoPhase(&F, PH_CALIBRATE); Flight_GoPhase(&F, PH_SETTLING);
    for (uint32_t i = 0u; i < FRAME_HZ; i++) { step(BOARD_G_MPS2, 0.0f, 0.0f); }
    TEST_ASSERT_EQUAL(PH_SETTLING, Flight_Phase(&F));
    for (uint32_t i = 0u; i < FRAME_HZ; i++) { step(8.0f * BOARD_G_MPS2, 0.0f, 0.0f); }
    TEST_ASSERT_EQUAL(PH_BOOST, Flight_Phase(&F));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_phase_sequence_nominal);
    RUN_TEST(test_no_false_landing_under_canopy);
    RUN_TEST(test_landing_after_shock_within_3s);
    RUN_TEST(test_landing_without_shock_or_gps_within_32s);
    RUN_TEST(test_shadow_mode_holds_canards);
    RUN_TEST(test_corrupt_word_in_flight_goes_cato_then_lands);
    RUN_TEST(test_liftoff_from_settling_is_sequenced);
    return UNITY_END();
}
