/* SYSID-01: the in-flight roll test schedule and its guards. */
#include "unity.h"
#include "rolltest.h"
#include "board.h"
#include <math.h>

#define HZ 200.0f
#define D2R 0.017453292519943295f
static RollTest R;
#define N0 ((int)lroundf(RT_T0_S * HZ))
#define NU ((int)lroundf(RT_UNIT_S * HZ))
#define NG ((int)lroundf(RT_GAP_S * HZ))
#define NH ((int)lroundf(RT_HOLD_S * HZ))

void setUp(void) { RollTest_Init(&R, HZ); }
void tearDown(void) {}

static RtEvent step(Phase ph, float q, float p) { return RollTest_Step(&R, ph, true, true, q, p); }

/* nothing before COAST; the full sequence with the documented timing and amplitudes */
static void test_nominal_sequence(void)
{
    for (int i = 0; i < 400; i++) { TEST_ASSERT_EQUAL_INT(RT_EV_NONE, step(PH_BOOST, 8000.0f, 0.0f)); TEST_ASSERT_EQUAL_FLOAT(0.0f, R.u_exc); }
    int n = 0; RtEvent ev = RT_EV_NONE;
    while (ev == RT_EV_NONE) { ev = step(PH_COAST, 6000.0f, 0.0f); n++; }
    TEST_ASSERT_EQUAL_INT(RT_EV_START, ev);
    TEST_ASSERT_EQUAL_INT(N0 + 1, n);                               /* the COAST entry frame + RT_T0_S */
    const float ua = 0.5f * (3000.0f / 6000.0f) * D2R;               /* torque-scaled, latched */
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, ua, R.u_exc);
    float u[7 * 200], pr[2 * 400];
    TEST_ASSERT_TRUE((NU <= 200) && (NH <= 400));
    u[0] = R.u_exc;
    for (int i = 1; i < 7 * NU; i++) { step(PH_COAST, 5000.0f, 0.0f); u[i] = R.u_exc; }   /* q falls: amplitude stays */
    const float want[7] = { 1, 1, 1, -1, -1, 1, -1 };
    for (int k = 0; k < 7; k++) { for (int j = 0; j < NU; j++) { TEST_ASSERT_FLOAT_WITHIN(1e-7f, want[k] * ua, u[k * NU + j]); } }
    for (int i = 0; i < NG; i++) { step(PH_COAST, 5000.0f, 0.0f); TEST_ASSERT_EQUAL_FLOAT(0.0f, R.u_exc); TEST_ASSERT_EQUAL_FLOAT(0.0f, R.p_ref); }
    for (int i = 0; i < 2 * NH; i++) { ev = step(PH_COAST, 4000.0f, 0.0f); pr[i] = R.p_ref; TEST_ASSERT_EQUAL_FLOAT(0.0f, R.u_exc);
                                    if (i < (2 * NH) - 1) { TEST_ASSERT_EQUAL_INT(RT_EV_NONE, ev); } }
    TEST_ASSERT_EQUAL_INT(RT_EV_DONE, ev);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, RT_RATE_DPS * D2R, pr[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, RT_RATE_DPS * D2R, pr[NH - 1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -RT_RATE_DPS * D2R, pr[NH]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -RT_RATE_DPS * D2R, pr[(2 * NH) - 1]);
    TEST_ASSERT_EQUAL_INT(RT_DOUBLET, R.st_end);
    for (int i = 0; i < 400; i++) { TEST_ASSERT_EQUAL_INT(RT_EV_NONE, step(PH_COAST, 4000.0f, 0.0f)); TEST_ASSERT_EQUAL_FLOAT(0.0f, R.p_ref); }
    TEST_ASSERT_EQUAL_INT(RT_DONE, R.st);                            /* once per flight */
}

/* the amplitude is clamped at low q, and a start below RT_Q_START_PA is skipped for good */
static void test_amplitude_clamp_and_skip(void)
{
    RtEvent ev = RT_EV_NONE;
    while (ev == RT_EV_NONE) { ev = step(PH_COAST, 3100.0f, 0.0f); }
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.5f * 3000.0f / 3100.0f * D2R, R.ua);
    RollTest_Init(&R, HZ);
    ev = RT_EV_NONE;
    while (ev == RT_EV_NONE) { ev = step(PH_COAST, 2900.0f, 0.0f); }
    TEST_ASSERT_EQUAL_INT(RT_EV_SKIP, ev); TEST_ASSERT_EQUAL_INT(RT_WHY_Q, R.why);
    for (int i = 0; i < 1000; i++) { TEST_ASSERT_EQUAL_INT(RT_EV_NONE, step(PH_COAST, 9000.0f, 0.0f)); TEST_ASSERT_EQUAL_FLOAT(0.0f, R.u_exc); }
    TEST_ASSERT_TRUE(fabsf(RT_UA_MAX_DEG * D2R) >= R.ua);
}

static void start(void) { RollTest_Init(&R, HZ); RtEvent ev = RT_EV_NONE; while (ev == RT_EV_NONE) { ev = step(PH_COAST, 6000.0f, 0.0f); } }

/* every guard abandons the test at once, with the outputs at zero */
static void test_guards_abort(void)
{
    start(); TEST_ASSERT_EQUAL_INT(RT_EV_NONE, step(PH_COAST, RT_Q_ABORT_PA + 1.0f, 0.0f));
    TEST_ASSERT_EQUAL_INT(RT_EV_ABORT, step(PH_COAST, RT_Q_ABORT_PA - 1.0f, 0.0f)); TEST_ASSERT_EQUAL_INT(RT_WHY_Q, R.why);
    TEST_ASSERT_EQUAL_INT(RT_EXC, R.st_end);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, R.u_exc); TEST_ASSERT_EQUAL_FLOAT(0.0f, R.p_ref);
    start(); TEST_ASSERT_EQUAL_INT(RT_EV_ABORT, step(PH_COAST, 6000.0f, 201.0f * D2R)); TEST_ASSERT_EQUAL_INT(RT_WHY_RATE, R.why);
    start(); TEST_ASSERT_EQUAL_INT(RT_EV_ABORT, step(PH_COAST, 6000.0f, -201.0f * D2R)); TEST_ASSERT_EQUAL_INT(RT_WHY_RATE, R.why);
    start(); TEST_ASSERT_EQUAL_INT(RT_EV_ABORT, RollTest_Step(&R, PH_COAST, false, true, 6000.0f, 0.0f)); TEST_ASSERT_EQUAL_INT(RT_WHY_FAULT, R.why);
    start(); TEST_ASSERT_EQUAL_INT(RT_EV_ABORT, RollTest_Step(&R, PH_COAST, true, false, 6000.0f, 0.0f)); TEST_ASSERT_EQUAL_INT(RT_WHY_Q, R.why);
    start(); TEST_ASSERT_EQUAL_INT(RT_EV_ABORT, step(PH_DESCENT, 6000.0f, 0.0f)); TEST_ASSERT_EQUAL_INT(RT_WHY_PHASE, R.why);
    start(); TEST_ASSERT_EQUAL_INT(RT_EV_ABORT, step(PH_COAST, NAN, 0.0f));
    start(); TEST_ASSERT_EQUAL_INT(RT_EV_ABORT, step(PH_COAST, 6000.0f, NAN));
    for (int i = 0; i < 1000; i++) { TEST_ASSERT_EQUAL_INT(RT_EV_NONE, step(PH_COAST, 6000.0f, 0.0f)); TEST_ASSERT_FALSE(RollTest_Active(&R)); }
}

/* the thresholds keep their order: start above abort, abort above the observer gate */
static void test_threshold_order(void)
{
    TEST_ASSERT_TRUE(RT_Q_START_PA > RT_Q_ABORT_PA);
#if CTRL_ROLL_LAW == 2
    TEST_ASSERT_TRUE(RT_Q_ABORT_PA > CTRL_ESO_Q_ON);
#endif
    TEST_ASSERT_TRUE(RT_P_ABORT_DPS > (1.5f * RT_RATE_DPS));
    TEST_ASSERT_TRUE(NU > 0);
}

/* a fault or an early apogee during the wait skips it */
static void test_wait_skips(void)
{
    RtEvent ev = RT_EV_NONE;
    step(PH_COAST, 6000.0f, 0.0f);
    for (int i = 0; (i < 200) && (ev == RT_EV_NONE); i++) { ev = RollTest_Step(&R, PH_COAST, false, true, 6000.0f, 0.0f); }
    TEST_ASSERT_EQUAL_INT(RT_EV_SKIP, ev); TEST_ASSERT_EQUAL_INT(RT_WHY_FAULT, R.why);
    RollTest_Init(&R, HZ);
    step(PH_COAST, 6000.0f, 0.0f);
    TEST_ASSERT_EQUAL_INT(RT_EV_SKIP, step(PH_DESCENT, 6000.0f, 0.0f));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nominal_sequence);
    RUN_TEST(test_amplitude_clamp_and_skip);
    RUN_TEST(test_guards_abort);
    RUN_TEST(test_wait_skips);
    RUN_TEST(test_threshold_order);
    return UNITY_END();
}
