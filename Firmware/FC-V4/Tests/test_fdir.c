#include "unity.h"
#include "fdir.h"
#include "board.h"
#include <math.h>

static Fdir F;
void setUp(void) { Fdir_Init(&F); }
void tearDown(void) {}

static void test_persistence_before_trip(void)
{
    const uint16_t n = Fdir_Config(MON_EST_COV)->n_persist;
    for (uint16_t i = 1u; i < n; i++) { TEST_ASSERT_FALSE(Fdir_Eval(&F, MON_EST_COV, 10.0f, false)); }
    TEST_ASSERT_TRUE(Fdir_Eval(&F, MON_EST_COV, 10.0f, false));
    TEST_ASSERT_EQUAL_UINT16(1u, F.trips[MON_EST_COV]);
}

static void test_interrupted_excursion_resets_count(void)
{
    const uint16_t n = Fdir_Config(MON_EST_COV)->n_persist;
    for (uint16_t i = 1u; i < n; i++) { (void)Fdir_Eval(&F, MON_EST_COV, 10.0f, false); }
    (void)Fdir_Eval(&F, MON_EST_COV, 0.01f, false);                 /* back in band */
    TEST_ASSERT_FALSE(Fdir_Eval(&F, MON_EST_COV, 10.0f, false));
}

static void test_soft_fault_clears(void)
{
    for (uint16_t i = 0u; i < FDIR_EST_COV_N; i++) { (void)Fdir_Eval(&F, MON_EST_COV, 10.0f, false); }
    TEST_ASSERT_EQUAL(SEV_SOFT, Fdir_Worst(&F, false));
    TEST_ASSERT_EQUAL(CAP_GYRO_ONLY, Fdir_Capability(&F));
    (void)Fdir_Eval(&F, MON_EST_COV, 0.01f, false);
    TEST_ASSERT_EQUAL(SEV_INFO, Fdir_Worst(&F, false));
    TEST_ASSERT_EQUAL(CAP_FULL, Fdir_Capability(&F));
}

static void test_hard_fault_latches(void)
{
    for (uint16_t i = 0u; i < FDIR_IMU_READ_N; i++) { (void)Fdir_Eval(&F, MON_IMU_READ, 1.0f, false); }
    TEST_ASSERT_EQUAL(SEV_HARD, Fdir_Worst(&F, false));
    TEST_ASSERT_EQUAL(CAP_NONE, Fdir_Capability(&F));
    (void)Fdir_Eval(&F, MON_IMU_READ, 0.0f, false);                  /* recovers ... */
    uint32_t l = 0u;
    TEST_ASSERT_TRUE(Fdir_LatchRead(&F, &l));
    TEST_ASSERT_EQUAL_HEX32(1u << MON_IMU_READ, l);                  /* ... but stays latched */
    TEST_ASSERT_EQUAL(SEV_HARD, Fdir_Worst(&F, false));
    TEST_ASSERT_EQUAL(CAP_NONE, Fdir_Capability(&F));
}

static void test_severity_depends_on_phase(void)
{
    for (uint16_t i = 0u; i < FDIR_OVERRUN_N; i++) { (void)Fdir_Eval(&F, MON_OVERRUN, 1.0f, true); }
    TEST_ASSERT_EQUAL(SEV_SOFT, Fdir_Worst(&F, true));               /* flight: no latch */
    uint32_t l = 1u;
    TEST_ASSERT_TRUE(Fdir_LatchRead(&F, &l));
    TEST_ASSERT_EQUAL_HEX32(0u, l);
    Fdir_Init(&F);
    for (uint16_t i = 0u; i < FDIR_OVERRUN_N; i++) { (void)Fdir_Eval(&F, MON_OVERRUN, 1.0f, false); }
    TEST_ASSERT_EQUAL(SEV_HARD, Fdir_Worst(&F, false));              /* pad: latch */
}

static void test_nan_residual_fails(void)
{
    TEST_ASSERT_TRUE(Fdir_Eval(&F, MON_EST_FINITE, NAN, false));     /* n_persist = 1; HARD on pad */
    TEST_ASSERT_EQUAL(SEV_HARD, Fdir_Worst(&F, false));
    TEST_ASSERT_EQUAL(CAP_GYRO_ONLY, Fdir_Capability(&F));
}

static void test_corrupt_latch_is_hard_and_no_capability(void)
{
    F.latch_inv ^= 0x10u;
    TEST_ASSERT_EQUAL(SEV_HARD, Fdir_Worst(&F, false));
    TEST_ASSERT_EQUAL(CAP_NONE, Fdir_Capability(&F));
}

static void test_info_monitor_never_degrades(void)
{
    TEST_ASSERT_TRUE(Fdir_Eval(&F, MON_GPS_LINK, 1.0f, false));
    TEST_ASSERT_EQUAL(SEV_INFO, Fdir_Worst(&F, false));
    TEST_ASSERT_EQUAL(CAP_FULL, Fdir_Capability(&F));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_persistence_before_trip);
    RUN_TEST(test_interrupted_excursion_resets_count);
    RUN_TEST(test_soft_fault_clears);
    RUN_TEST(test_hard_fault_latches);
    RUN_TEST(test_severity_depends_on_phase);
    RUN_TEST(test_nan_residual_fails);
    RUN_TEST(test_corrupt_latch_is_hard_and_no_capability);
    RUN_TEST(test_info_monitor_never_degrades);
    return UNITY_END();
}
