/* CAL-01: canard zero alignment session (gating, limits, exits from ALIGN). */
#include "unity.h"
#include "align.h"
#include <string.h>

static Align A;
static const int32_t STORED[4] = { 100, -200, 300, -400 };

void setUp(void) { Align_Init(&A); }
void tearDown(void) {}

static void begin_ok(Phase from) { TEST_ASSERT_EQUAL_INT(AL_OK, Align_Begin(&A, STORED, from, false)); }

/* from SETTLING or READY only, never with a HARD fault, never twice */
static void test_begin_gating(void)
{
    const Phase refused[] = { PH_BOOT, PH_CALIBRATE, PH_BOOST, PH_COAST, PH_DESCENT, PH_LANDED,
                              PH_LOCKOUT, PH_CATO_SAFE, PH_ALIGN };
    for (uint32_t i = 0u; i < sizeof refused / sizeof refused[0]; i++) {
        TEST_ASSERT_EQUAL_INT(AL_ERR_PHASE, Align_Begin(&A, STORED, refused[i], false));
        TEST_ASSERT_FALSE(Align_Active(&A));
    }
    TEST_ASSERT_EQUAL_INT(AL_ERR_FAULT, Align_Begin(&A, STORED, PH_READY, true));
    begin_ok(PH_READY);
    TEST_ASSERT_EQUAL_INT(AL_ERR_ACTIVE, Align_Begin(&A, STORED, PH_SETTLING, false));
    Align_Init(&A);
    begin_ok(PH_SETTLING);
}

/* nothing moves without a session */
static void test_commands_need_session(void)
{
    TEST_ASSERT_EQUAL_INT(AL_ERR_IDLE, Align_Jog(&A, 0u, 100));
    TEST_ASSERT_EQUAL_INT(AL_ERR_IDLE, Align_Set(&A, 0u, 100));
    TEST_ASSERT_EQUAL_PTR(STORED, Align_Zero(&A, STORED));
}

/* jogs accumulate on the trial zero; the zero reaches +/-90 deg; out-of-range
 * input is refused, not clamped */
static void test_jog_set_limits(void)
{
    begin_ok(PH_SETTLING);
    TEST_ASSERT_EQUAL_INT(AL_OK, Align_Jog(&A, 2u, -50));
    TEST_ASSERT_EQUAL_INT32(250, Align_Zero(&A, STORED)[2]);
    TEST_ASSERT_EQUAL_INT(AL_ERR_FIN, Align_Jog(&A, 4u, 10));
    TEST_ASSERT_EQUAL_INT(AL_OK, Align_Jog(&A, 1u, -10000));
    TEST_ASSERT_EQUAL_INT(AL_ERR_STEP, Align_Jog(&A, 0u, 10001));
    TEST_ASSERT_EQUAL_INT(AL_OK, Align_Set(&A, 0u, 89900));
    TEST_ASSERT_EQUAL_INT(AL_OK, Align_Set(&A, 3u, -90000));
    TEST_ASSERT_EQUAL_INT(AL_ERR_RANGE, Align_Jog(&A, 0u, 200));
    TEST_ASSERT_EQUAL_INT32(89900, A.trial[0]);                 /* unchanged after a refusal */
    TEST_ASSERT_EQUAL_INT(AL_ERR_RANGE, Align_Set(&A, 1u, -90001));
}

/* save keeps the trial; cancel reverts to stored */
static void test_save_keeps_cancel_reverts(void)
{
    begin_ok(PH_SETTLING);
    (void)Align_Jog(&A, 1u, 500);
    const int32_t *z = Align_End(&A, AL_END_SAVED);
    TEST_ASSERT_EQUAL_INT32(300, z[1]);
    TEST_ASSERT_FALSE(Align_Active(&A));

    begin_ok(PH_READY);
    (void)Align_Jog(&A, 1u, 500);
    z = Align_End(&A, AL_END_CANCEL);
    TEST_ASSERT_EQUAL_INT32(-200, z[1]);
    TEST_ASSERT_EQUAL_PTR(STORED, Align_Zero(&A, STORED));
}

/* the session lasts as long as the ALIGN phase, with no timeout; leaving it
 * any other way reverts, with the reason */
static void test_poll_exits(void)
{
    begin_ok(PH_SETTLING);
    (void)Align_Jog(&A, 0u, 1000);
    for (uint32_t k = 0u; k < 100000u; k++) { TEST_ASSERT_EQUAL_INT(AL_END_NONE, Align_Poll(&A, PH_ALIGN)); }
    TEST_ASSERT_TRUE(Align_Active(&A));
    const struct { Phase ph; AlignEnd want; } c[] = {
        { PH_LOCKOUT, AL_END_FAULT }, { PH_BOOST, AL_END_LIFTOFF }, { PH_SETTLING, AL_END_PHASE },
        { PH_READY, AL_END_PHASE }, { PH_CATO_SAFE, AL_END_FAULT },
    };
    for (uint32_t i = 0u; i < sizeof c / sizeof c[0]; i++) {
        Align_Init(&A);
        begin_ok(PH_SETTLING);
        (void)Align_Jog(&A, 0u, 1000);
        TEST_ASSERT_EQUAL_INT(c[i].want, Align_Poll(&A, c[i].ph));
        TEST_ASSERT_FALSE(Align_Active(&A));
        TEST_ASSERT_EQUAL_INT32(100, A.trial[0]);               /* reverted */
    }
    TEST_ASSERT_EQUAL_INT(AL_END_NONE, Align_Poll(&A, PH_SETTLING));   /* idle: nothing to end */
}

static void test_parse_mdeg(void)
{
    const struct { const char *s; bool ok; int32_t v; } c[] = {
        { "0.1", true, 100 }, { "-0.125", true, -125 }, { "+2", true, 2000 }, { ".5", true, 500 },
        { "-.02", true, -20 }, { "45", true, 45000 }, { "-90", true, -90000 }, { "99.999", true, 99999 },
        { "0.1234", false, 0 }, { "100", false, 0 }, { "", false, 0 }, { "-", false, 0 }, { ".", false, 0 },
        { "1e3", false, 0 }, { "1..2", false, 0 }, { "abc", false, 0 }, { "1.2.3", false, 0 },
    };
    for (uint32_t i = 0u; i < sizeof c / sizeof c[0]; i++) {
        int32_t v = 12345;
        TEST_ASSERT_EQUAL_MESSAGE(c[i].ok, Align_ParseMdeg(c[i].s, &v), c[i].s);
        if (c[i].ok) { TEST_ASSERT_EQUAL_INT32_MESSAGE(c[i].v, v, c[i].s); }
    }
    TEST_ASSERT_FALSE(Align_ParseMdeg(0, 0));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_begin_gating);
    RUN_TEST(test_commands_need_session);
    RUN_TEST(test_jog_set_limits);
    RUN_TEST(test_save_keeps_cancel_reverts);
    RUN_TEST(test_poll_exits);
    RUN_TEST(test_parse_mdeg);
    return UNITY_END();
}
