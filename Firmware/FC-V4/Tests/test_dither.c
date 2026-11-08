/* ADR-004 §3.6: the roll-law dither (Matlab P.Dither) as flown. */
#include "unity.h"
#include "control.h"
#include "board.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

static Controller C;

/* +a +a -a -a ... at the frame rate, zero mean over a period, amplitude 1.5 x dead band */
static void test_dither_square_wave_zero_mean(void)
{
    Ctrl_Init(&C); Ctrl_SetMode(&C, CTRL_MODE_FULL); Ctrl_SetQbar(&C, 6000.0f, true);
    float sum = 0.0f;
    for (uint32_t k = 0u; k < 4u * CTRL_DITHER_PERIOD_N; k++) {
        const float d = Ctrl_RollLaw(&C, 0.0f, FRAME_DT_S, false);
        TEST_ASSERT_FLOAT_WITHIN(1e-7f, CTRL_DITHER_AMP_RAD, fabsf(d));
        TEST_ASSERT_EQUAL_FLOAT(((k / (CTRL_DITHER_PERIOD_N / 2u)) % 2u == 0u) ? CTRL_DITHER_AMP_RAD : -CTRL_DITHER_AMP_RAD, d);
        sum += d;
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, sum);
}

/* the dither must not drive the trim integrator (it is added after it) */
static void test_dither_does_not_wind_trim(void)
{
    Ctrl_Init(&C); Ctrl_SetMode(&C, CTRL_MODE_FULL); Ctrl_SetQbar(&C, 6000.0f, true);
    for (uint32_t k = 0u; k < 2000u; k++) { (void)Ctrl_RollLaw(&C, 0.0f, FRAME_DT_S, true); }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, C.trim);
}

/* off -> no dither, and the phase restarts on re-entry */
static void test_dither_stops_when_off(void)
{
    Ctrl_Init(&C); Ctrl_SetMode(&C, CTRL_MODE_FULL);
    (void)Ctrl_RollLaw(&C, 0.0f, FRAME_DT_S, false);
    Ctrl_SetMode(&C, CTRL_MODE_OFF);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, C.dither);
    TEST_ASSERT_EQUAL_UINT32(0u, C.dither_k);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_dither_square_wave_zero_mean);
    RUN_TEST(test_dither_does_not_wind_trim);
    RUN_TEST(test_dither_stops_when_off);
    return UNITY_END();
}
