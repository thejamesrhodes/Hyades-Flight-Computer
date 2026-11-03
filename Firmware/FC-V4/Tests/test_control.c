#include "unity.h"
#include "control.h"
#include "board.h"

static Controller C;
static const float Q_ID[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
static const float ZERO3[3] = { 0.0f, 0.0f, 0.0f };

void setUp(void) { Ctrl_Init(&C); }
void tearDown(void) {}

static void test_starts_off_with_centred_fins(void)
{
    float fin[4] = { 9.0f, 9.0f, 9.0f, 9.0f };
    const float w[3] = { 1.0f, 2.0f, 3.0f };
    TEST_ASSERT_EQUAL(CTRL_MODE_OFF, C.mode);
    Ctrl_Update(&C, Q_ID, w, w, FRAME_DT_S, fin);
    for (int i = 0; i < 4; i++) { TEST_ASSERT_EQUAL_FLOAT(0.0f, fin[i]); }
}

static void test_rate_damper_opposes_roll_rate(void)
{
    float fin[4];
    const float w[3] = { 0.0f, 0.0f, 1.0f };                    /* +1 rad/s roll */
    Ctrl_SetMode(&C, CTRL_MODE_RATE_DAMP);
    Ctrl_Update(&C, Q_ID, ZERO3, w, FRAME_DT_S, fin);
    for (int i = 0; i < 4; i++) { TEST_ASSERT_FLOAT_WITHIN(1e-6f, -CTRL_RD_GAIN_ROLL, fin[i]); }
}

static void test_rate_damper_uses_gyro_not_estimator(void)
{
    float fin[4];
    const float w_est[3] = { 0.0f, 0.0f, 50.0f };               /* garbage estimate */
    Ctrl_SetMode(&C, CTRL_MODE_RATE_DAMP);
    Ctrl_Update(&C, Q_ID, w_est, ZERO3, FRAME_DT_S, fin);
    for (int i = 0; i < 4; i++) { TEST_ASSERT_EQUAL_FLOAT(0.0f, fin[i]); }
}

static void test_rate_damper_saturates(void)
{
    float fin[4];
    const float w[3] = { 0.0f, 0.0f, -100.0f };
    Ctrl_SetMode(&C, CTRL_MODE_RATE_DAMP);
    Ctrl_Update(&C, Q_ID, ZERO3, w, FRAME_DT_S, fin);
    for (int i = 0; i < 4; i++) { TEST_ASSERT_EQUAL_FLOAT(CTRL_FIN_LIMIT, fin[i]); }
}

static void test_pitch_yaw_disabled(void)
{
    float fin[4];
    const float w[3] = { 3.0f, -3.0f, 0.0f };
    Ctrl_SetMode(&C, CTRL_MODE_FULL);
    for (int k = 0; k < 50; k++) { Ctrl_Update(&C, Q_ID, w, w, FRAME_DT_S, fin); }
    for (int i = 0; i < 4; i++) { TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, fin[i]); }
}

static void test_full_mode_damps_roll_rate(void)
{
    float fin[4];
    const float w[3] = { 0.0f, 0.0f, 1.0f };
    Ctrl_SetMode(&C, CTRL_MODE_FULL);
    for (int k = 0; k < 20; k++) { Ctrl_Update(&C, Q_ID, w, w, FRAME_DT_S, fin); }
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_TRUE(fin[i] < 0.0f);                         /* opposes +roll */
        TEST_ASSERT_TRUE(fin[i] >= -CTRL_FIN_LIMIT);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, fin[0], fin[i]);         /* pure roll: common mode */
    }
}

static void test_full_mode_quiescent_is_zero(void)
{
    float fin[4];
    Ctrl_SetMode(&C, CTRL_MODE_FULL);
    for (int k = 0; k < 100; k++) { Ctrl_Update(&C, Q_ID, ZERO3, ZERO3, FRAME_DT_S, fin); }
    for (int i = 0; i < 4; i++) { TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, fin[i]); }
}

static void test_bumpless_entry_preloads_actuator_filter(void)
{
    float fin[4];
    const float w[3] = { 0.0f, 0.0f, 1.0f };
    Ctrl_SetMode(&C, CTRL_MODE_RATE_DAMP);
    Ctrl_Update(&C, Q_ID, w, w, FRAME_DT_S, fin);
    Ctrl_SetMode(&C, CTRL_MODE_FULL);
    for (int i = 0; i < 4; i++) { TEST_ASSERT_FLOAT_WITHIN(1e-6f, fin[i], C.delta_f[i]); }
    /* Steady-state preload: a constant input reproduces itself. */
    Biquad s = C.bq_delta[0];
    const float u = fin[0];
    float y = 0.0f;
    for (int k = 0; k < 5; k++) {
        y = C.b0 * u + s.z1;
        s.z1 = C.b1 * u - C.a1 * y + s.z2;
        s.z2 = C.b2 * u - C.a2 * y;
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, u, y);
    }
}

static void test_mode_change_resets_integral(void)
{
    C.e_I[2] = 0.2f;
    Ctrl_SetMode(&C, CTRL_MODE_FULL);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, C.e_I[2]);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_off_with_centred_fins);
    RUN_TEST(test_rate_damper_opposes_roll_rate);
    RUN_TEST(test_rate_damper_uses_gyro_not_estimator);
    RUN_TEST(test_rate_damper_saturates);
    RUN_TEST(test_pitch_yaw_disabled);
    RUN_TEST(test_full_mode_damps_roll_rate);
    RUN_TEST(test_full_mode_quiescent_is_zero);
    RUN_TEST(test_bumpless_entry_preloads_actuator_filter);
    RUN_TEST(test_mode_change_resets_integral);
    return UNITY_END();
}
