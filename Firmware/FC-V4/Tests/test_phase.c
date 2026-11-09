#include "unity.h"
#include "phase.h"

static PhaseWord W;
static const Phase k_all[PHASE_COUNT] = {
    PH_BOOT, PH_CALIBRATE, PH_SETTLING, PH_READY, PH_BOOST,
    PH_COAST, PH_DESCENT, PH_LANDED, PH_LOCKOUT, PH_CATO_SAFE, PH_ALIGN
};

void setUp(void) { Phase_Init(&W); }
void tearDown(void) {}

static Phase now(void) { Phase p = PH_LOCKOUT; TEST_ASSERT_TRUE(Phase_Read(&W, &p)); return p; }

static void test_codes_have_hamming_distance_6(void)
{
    for (uint32_t i = 0u; i < PHASE_COUNT; i++) {
        for (uint32_t j = i + 1u; j < PHASE_COUNT; j++) {
            const uint32_t d = (uint32_t)__builtin_popcount((unsigned)k_all[i] ^ (unsigned)k_all[j]);
            TEST_ASSERT_GREATER_OR_EQUAL_UINT32(6u, d);
        }
        TEST_ASSERT_EQUAL_UINT8(i, Phase_Index(k_all[i]));
    }
}

static void test_nominal_flight_sequence(void)
{
    const Phase seq[] = { PH_CALIBRATE, PH_SETTLING, PH_READY, PH_BOOST, PH_COAST, PH_DESCENT, PH_LANDED };
    for (uint32_t i = 0u; i < sizeof seq / sizeof seq[0]; i++) {
        TEST_ASSERT_TRUE(Phase_Transition(&W, seq[i]));
        TEST_ASSERT_EQUAL_HEX16(seq[i], now());
    }
}

static void test_pad_demotion_only_backwards_edge(void)
{
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_CALIBRATE));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_SETTLING));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_READY));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_SETTLING));   /* demote */
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_CALIBRATE)); /* no other way back */
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_LANDED));    /* no skipping to recovery phases */
    TEST_ASSERT_EQUAL_HEX16(PH_SETTLING, now());
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_BOOST));      /* STM-01 F3: liftoff while not READY is sequenced */
}

static void test_no_flight_backwards_or_skips(void)
{
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_BOOST));     /* BOOT -> BOOST */
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_CALIBRATE));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_SETTLING));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_READY));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_BOOST));
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_READY));
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_LOCKOUT));   /* in flight: CATO_SAFE, not LOCKOUT */
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_CATO_SAFE));
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_COAST));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_LANDED));
}

static void test_lockout_is_terminal(void)
{
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_LOCKOUT));
    for (uint32_t i = 0u; i < PHASE_COUNT; i++) { TEST_ASSERT_FALSE(Phase_Transition(&W, k_all[i])); }
    TEST_ASSERT_EQUAL_HEX16(PH_LOCKOUT, now());
}

static void test_corruption_detected_and_refused(void)
{
    W.v ^= 0x4u;                                            /* single-bit upset */
    Phase p;
    TEST_ASSERT_FALSE(Phase_Read(&W, &p));
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_CALIBRATE));
    Phase_ForceSafe(&W, false);
    TEST_ASSERT_EQUAL_HEX16(PH_LOCKOUT, now());
    Phase_ForceSafe(&W, true);
    TEST_ASSERT_EQUAL_HEX16(PH_CATO_SAFE, now());
}

static void test_consistent_but_invalid_code_rejected(void)
{
    W.v = 0x1234u; W.inv = ~0x1234u;
    Phase p;
    TEST_ASSERT_FALSE(Phase_Read(&W, &p));
    TEST_ASSERT_EQUAL_STRING("INVALID", Phase_Name((Phase)0x1234u));
}

static void test_actuation_policy(void)
{
    TEST_ASSERT_TRUE(Phase_ActuationAllowed(PH_BOOST, false));
    TEST_ASSERT_TRUE(Phase_ActuationAllowed(PH_COAST, false));
    TEST_ASSERT_FALSE(Phase_ActuationAllowed(PH_READY, false));
    TEST_ASSERT_TRUE(Phase_ActuationAllowed(PH_READY, true));
    TEST_ASSERT_FALSE(Phase_ActuationAllowed(PH_SETTLING, true));
    TEST_ASSERT_FALSE(Phase_ActuationAllowed(PH_DESCENT, true));
    TEST_ASSERT_FALSE(Phase_ActuationAllowed(PH_LOCKOUT, true));
    TEST_ASSERT_TRUE(Phase_InFlight(PH_DESCENT));
    TEST_ASSERT_FALSE(Phase_InFlight(PH_CATO_SAFE));
}

static void test_recover_only_from_lockout(void)
{
    TEST_ASSERT_FALSE(Phase_Recover(&W));                            /* BOOT */
    Phase_ForceSafe(&W, true);                                       /* CATO_SAFE */
    TEST_ASSERT_FALSE(Phase_Recover(&W));
    TEST_ASSERT_EQUAL_HEX16(PH_CATO_SAFE, now());
    Phase_ForceSafe(&W, false);                                      /* LOCKOUT */
    TEST_ASSERT_TRUE(Phase_Recover(&W));
    TEST_ASSERT_EQUAL_HEX16(PH_SETTLING, now());
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_COAST));               /* no skipping BOOST */
    W.v ^= 0x4u;                                                     /* corrupt word */
    TEST_ASSERT_TRUE(Phase_Recover(&W));
    TEST_ASSERT_EQUAL_HEX16(PH_SETTLING, now());
}


/* CAL-01: ALIGN is entered from SETTLING or READY only; it is left only to
 * SETTLING (save/cancel), BOOST (liftoff, F3) or LOCKOUT (HARD); never READY,
 * and it never actuates and is never "in flight". */
static void test_align_phase_edges(void)
{
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_ALIGN));     /* not from BOOT */
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_CALIBRATE));
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_ALIGN));     /* not from CALIBRATE */
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_SETTLING));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_ALIGN));
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_READY));     /* never straight to READY */
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_CALIBRATE));
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_CATO_SAFE));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_SETTLING));   /* save / cancel */
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_READY));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_ALIGN));      /* from READY too */
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_BOOST));      /* liftoff while aligning is sequenced */
    TEST_ASSERT_FALSE(Phase_Transition(&W, PH_ALIGN));     /* never in flight */
    Phase_Init(&W);
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_CALIBRATE));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_SETTLING));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_ALIGN));
    TEST_ASSERT_TRUE(Phase_Transition(&W, PH_LOCKOUT));    /* HARD on the pad */
    TEST_ASSERT_FALSE(Phase_InFlight(PH_ALIGN));
    TEST_ASSERT_FALSE(Phase_ActuationAllowed(PH_ALIGN, true));
    TEST_ASSERT_EQUAL_STRING("ALIGN", Phase_Name(PH_ALIGN));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_codes_have_hamming_distance_6);
    RUN_TEST(test_nominal_flight_sequence);
    RUN_TEST(test_pad_demotion_only_backwards_edge);
    RUN_TEST(test_no_flight_backwards_or_skips);
    RUN_TEST(test_lockout_is_terminal);
    RUN_TEST(test_corruption_detected_and_refused);
    RUN_TEST(test_consistent_but_invalid_code_rejected);
    RUN_TEST(test_actuation_policy);
    RUN_TEST(test_recover_only_from_lockout);
    RUN_TEST(test_align_phase_edges);
    return UNITY_END();
}
