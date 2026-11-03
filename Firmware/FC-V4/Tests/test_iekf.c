#include "unity.h"
#include "iekf.h"
#include "board.h"
#include <math.h>

static IEKF_State S;
static const float NOMAG[3] = { 0.0f, 0.0f, 0.0f };

void setUp(void) { IEKF_Init(&S); }
void tearDown(void) {}

/* Body-frame specific force at rest for a body->world rotation about X by a (rad). */
static void accel_tilted_x(float a, float out[3])
{
    out[0] = 0.0f;
    out[1] = BOARD_G_MPS2 * sinf(a);
    out[2] = BOARD_G_MPS2 * cosf(a);
}

static void test_seed_level(void)
{
    const float acc[3] = { 0.0f, 0.0f, BOARD_G_MPS2 };
    float r, p, y;
    IEKF_Seed(&S, acc, NOMAG);
    IEKF_GetEuler(&S, &r, &p, &y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, r);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, p);
}

static void test_seed_tilted_roll(void)
{
    float acc[3], r, p, y;
    accel_tilted_x(0.5f, acc);                      /* ~28.65 deg */
    IEKF_Seed(&S, acc, NOMAG);
    IEKF_GetEuler(&S, &r, &p, &y);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 28.6479f, r);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, p);
}

static void test_propagation_integrates_rate(void)
{
    const float acc[3] = { 0.0f, 0.0f, BOARD_G_MPS2 };
    IEKF_Seed(&S, acc, NOMAG);
    const float w_dps[3] = { 0.0f, 0.0f, 90.0f };
    for (uint32_t k = 0u; k < FRAME_HZ; k++) { IEKF_Predict(&S, w_dps, FRAME_DT_S); }   /* 1 s */
    float r, p, y;
    IEKF_GetEuler(&S, &r, &p, &y);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 90.0f, y);
    const float n = sqrtf(S.q[0]*S.q[0] + S.q[1]*S.q[1] + S.q[2]*S.q[2] + S.q[3]*S.q[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, n);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 90.0f * 0.0174533f, S.omega[2]);
}

static void test_accel_updates_correct_tilt_error(void)
{
    float acc[3];
    accel_tilted_x(0.0f, acc);
    IEKF_Seed(&S, acc, NOMAG);
    accel_tilted_x(0.17f, acc);                     /* truth now ~9.7 deg roll */
    const float w0[3] = { 0.0f, 0.0f, 0.0f };
    for (uint32_t k = 0u; k < 10 * FRAME_HZ; k++) {      /* 10 s at rest */
        IEKF_Predict(&S, w0, FRAME_DT_S);
        (void)IEKF_UpdateAccel(&S, acc);
    }
    float r, p, y;
    IEKF_GetEuler(&S, &r, &p, &y);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 9.74f, r);
    TEST_ASSERT_TRUE(IEKF_IsFinite(&S));
    /* Tilt variance collapses well inside the FDIR bound, while heading
     * (unobservable without a magnetometer) keeps its prior. */
    TEST_ASSERT_TRUE(S.P[0] < 0.01f * IEKF_P0_ANGLE);
    TEST_ASSERT_TRUE(IEKF_TiltVar(&S) < 0.1f * FDIR_EST_TILT_VAR_MAX);
    for (int i = 0; i < IEKF_N; i++) {               /* symmetric, positive diagonal */
        TEST_ASSERT_TRUE(S.P[i*IEKF_N+i] > 0.0f);
        for (int j = 0; j < IEKF_N; j++) { TEST_ASSERT_EQUAL_FLOAT(S.P[i*IEKF_N+j], S.P[j*IEKF_N+i]); }
    }
}

static void test_observable_gyro_bias_converges(void)
{
    /* True bias on X (observable from gravity when level). Gyro reads bias
     * while the body is still; the filter should learn it. */
    const float acc[3] = { 0.0f, 0.0f, BOARD_G_MPS2 };
    IEKF_Seed(&S, acc, NOMAG);
    const float bias_dps = 0.5f;
    const float meas[3] = { bias_dps, 0.0f, 0.0f };
    for (uint32_t k = 0u; k < 120 * FRAME_HZ; k++) {     /* 120 s */
        IEKF_Predict(&S, meas, FRAME_DT_S);
        (void)IEKF_UpdateAccel(&S, acc);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f * bias_dps * 0.0174533f, bias_dps * 0.0174533f, S.bias[0]);  /* rad/s */
    float r, p, y;
    IEKF_GetEuler(&S, &r, &p, &y);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, r);
}

static void test_tilt_var_excludes_heading_growth(void)
{
    /* Tilted 60 deg, 65 s of accel updates: heading (about gravity) keeps its
     * prior and grows with the unobservable bias; tilt variance must exclude it. */
    float acc[3];
    accel_tilted_x(1.047f, acc);
    IEKF_Seed(&S, acc, NOMAG);
    for (uint32_t k = 0u; k < 5u * FRAME_HZ; k++) { IEKF_Predict(&S, NOMAG, FRAME_DT_S); (void)IEKF_UpdateAccel(&S, acc); }
    TEST_ASSERT_TRUE(IEKF_TiltVar(&S) < FDIR_EST_TILT_VAR_MAX);
    for (uint32_t k = 0u; k < 60u * FRAME_HZ; k++) { IEKF_Predict(&S, NOMAG, FRAME_DT_S); (void)IEKF_UpdateAccel(&S, acc); }
    const float tr = S.P[0] + S.P[IEKF_N+1] + S.P[2*IEKF_N+2];
    TEST_ASSERT_TRUE(tr > 0.9f * IEKF_P0_ANGLE);             /* heading keeps its prior (+ growth) */
    TEST_ASSERT_TRUE(IEKF_TiltVar(&S) < 0.1f * FDIR_EST_TILT_VAR_MAX);
}

static void test_seed_prior_fails_tilt_gate_until_converged(void)
{
    TEST_ASSERT_TRUE(IEKF_TiltVar(&S) > FDIR_EST_TILT_VAR_MAX);
}

static void test_accel_gate_rejects_high_g(void)
{
    const float acc[3] = { 0.0f, 0.0f, BOARD_G_MPS2 };
    IEKF_Seed(&S, acc, NOMAG);
    const float boost[3] = { 0.0f, 0.0f, 5.0f * BOARD_G_MPS2 };
    TEST_ASSERT_FALSE(IEKF_UpdateAccel(&S, boost));
    TEST_ASSERT_TRUE(IEKF_UpdateAccel(&S, acc));
}

static void test_nan_detected(void)
{
    S.P[7] = NAN;
    TEST_ASSERT_FALSE(IEKF_IsFinite(&S));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_seed_level);
    RUN_TEST(test_seed_tilted_roll);
    RUN_TEST(test_propagation_integrates_rate);
    RUN_TEST(test_accel_updates_correct_tilt_error);
    RUN_TEST(test_observable_gyro_bias_converges);
    RUN_TEST(test_tilt_var_excludes_heading_growth);
    RUN_TEST(test_seed_prior_fails_tilt_gate_until_converged);
    RUN_TEST(test_accel_gate_rejects_high_g);
    RUN_TEST(test_nan_detected);
    return UNITY_END();
}
