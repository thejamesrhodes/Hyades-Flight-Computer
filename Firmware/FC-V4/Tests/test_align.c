/* CAL-01: canard zero alignment session (gating, limits, auto-revert). */
#include "unity.h"
#include "align.h"
#include <string.h>

static Align A;
static const int16_t STORED[4] = { 100, -200, 300, -400 };

void setUp(void) { Align_Init(&A); }
void tearDown(void) {}

static void begin_ok(uint32_t t) { TEST_ASSERT_EQUAL_INT(AL_OK, Align_Begin(&A, STORED, PH_SETTLING, false, t)); }

/* only on the pad before READY, never with a HARD fault, never twice */
static void test_begin_gating(void)
{
    const Phase refused[] = { PH_READY, PH_BOOST, PH_COAST, PH_DESCENT, PH_LANDED, PH_LOCKOUT, PH_CATO_SAFE };
    for (uint32_t i = 0u; i < sizeof refused / sizeof refused[0]; i++) {
        TEST_ASSERT_EQUAL_INT(AL_ERR_PHASE, Align_Begin(&A, STORED, refused[i], false, 0u));
        TEST_ASSERT_FALSE(Align_Active(&A));
    }
    TEST_ASSERT_EQUAL_INT(AL_ERR_FAULT, Align_Begin(&A, STORED, PH_SETTLING, true, 0u));
    begin_ok(0u);
    TEST_ASSERT_EQUAL_INT(AL_ERR_ACTIVE, Align_Begin(&A, STORED, PH_SETTLING, false, 0u));
}

/* nothing moves without a session */
static void test_commands_need_session(void)
{
    TEST_ASSERT_EQUAL_INT(AL_ERR_IDLE, Align_Jog(&A, 0u, 100, 0u));
    TEST_ASSERT_EQUAL_INT(AL_ERR_IDLE, Align_Set(&A, 0u, 100, 0u));
    TEST_ASSERT_EQUAL_PTR(STORED, Align_Zero(&A, STORED));
}

/* jogs accumulate on the trial zero; out-of-range input is refused, not clamped */
static void test_jog_set_limits(void)
{
    begin_ok(0u);
    TEST_ASSERT_EQUAL_INT(AL_OK, Align_Jog(&A, 2u, -50, 1u));
    TEST_ASSERT_EQUAL_INT16(250, Align_Zero(&A, STORED)[2]);
    TEST_ASSERT_EQUAL_INT(AL_ERR_FIN, Align_Jog(&A, 4u, 10, 1u));
    TEST_ASSERT_EQUAL_INT(AL_ERR_STEP, Align_Jog(&A, 0u, 2001, 1u));
    TEST_ASSERT_EQUAL_INT(AL_OK, Align_Set(&A, 0u, 9900, 1u));
    TEST_ASSERT_EQUAL_INT(AL_ERR_RANGE, Align_Jog(&A, 0u, 200, 1u));
    TEST_ASSERT_EQUAL_INT16(9900, A.trial[0]);                 /* unchanged after a refusal */
    TEST_ASSERT_EQUAL_INT(AL_ERR_RANGE, Align_Set(&A, 1u, -10001, 1u));
}

/* save keeps the trial; cancel and every automatic end revert to stored */
static void test_save_keeps_cancel_reverts(void)
{
    begin_ok(0u);
    (void)Align_Jog(&A, 1u, 500, 1u);
    const int16_t *z = Align_End(&A, AL_END_SAVED);
    TEST_ASSERT_EQUAL_INT16(300, z[1]);
    TEST_ASSERT_FALSE(Align_Active(&A));

    begin_ok(2u);
    (void)Align_Jog(&A, 1u, 500, 3u);
    z = Align_End(&A, AL_END_CANCEL);
    TEST_ASSERT_EQUAL_INT16(-200, z[1]);
    TEST_ASSERT_EQUAL_PTR(STORED, Align_Zero(&A, STORED));
}

static void test_auto_end_reasons(void)
{
    struct { Phase ph; bool hard; bool link; uint32_t t; AlignEnd want; } c[] = {
        { PH_READY,    false, true,  10u,             AL_END_PHASE },
        { PH_LOCKOUT,  false, true,  10u,             AL_END_PHASE },
        { PH_SETTLING, true,  true,  10u,             AL_END_FAULT },
        { PH_SETTLING, false, false, 10u,             AL_END_LINK },
        { PH_SETTLING, false, true,  ALIGN_IDLE_MS,   AL_END_TIMEOUT },
    };
    for (uint32_t i = 0u; i < sizeof c / sizeof c[0]; i++) {
        Align_Init(&A);
        begin_ok(0u);
        (void)Align_Jog(&A, 0u, 1000, 0u);
        TEST_ASSERT_EQUAL_INT(c[i].want, Align_Poll(&A, c[i].ph, c[i].hard, c[i].link, c[i].t));
        TEST_ASSERT_FALSE(Align_Active(&A));
        TEST_ASSERT_EQUAL_INT16(100, A.trial[0]);               /* reverted */
    }
}

/* a command restarts the idle timer; polling alone does not */
static void test_idle_timer(void)
{
    begin_ok(0u);
    TEST_ASSERT_EQUAL_INT(AL_END_NONE, Align_Poll(&A, PH_SETTLING, false, true, ALIGN_IDLE_MS - 1u));
    (void)Align_Jog(&A, 0u, 10, ALIGN_IDLE_MS - 1u);
    TEST_ASSERT_EQUAL_INT(AL_END_NONE, Align_Poll(&A, PH_SETTLING, false, true, (2u * ALIGN_IDLE_MS) - 2u));
    TEST_ASSERT_EQUAL_UINT32(1u, Align_IdleLeftMs(&A, (2u * ALIGN_IDLE_MS) - 2u));
    TEST_ASSERT_EQUAL_INT(AL_END_TIMEOUT, Align_Poll(&A, PH_SETTLING, false, true, (2u * ALIGN_IDLE_MS) - 1u));
}

static void test_parse_mdeg(void)
{
    const struct { const char *s; bool ok; int32_t v; } c[] = {
        { "0.1", true, 100 }, { "-0.125", true, -125 }, { "+2", true, 2000 }, { ".5", true, 500 },
        { "-.02", true, -20 }, { "10", true, 10000 }, { "99.999", true, 99999 },
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
    RUN_TEST(test_auto_end_reasons);
    RUN_TEST(test_idle_timer);
    RUN_TEST(test_parse_mdeg);
    return UNITY_END();
}
