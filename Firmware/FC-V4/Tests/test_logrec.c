#include "unity.h"
#include "logrec.h"
#include <math.h>
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

static LogState sample(void)
{
    LogState s;
    memset(&s, 0, sizeof s);
    s.t_us = 123456789u; s.seq = 4242u; s.phase = 3u; s.mode = 2u; s.flags = 0x13u;
    s.q[0] = 0.7071f; s.q[1] = 0.0f; s.q[2] = -0.7071f; s.q[3] = 0.001f;
    s.w_dps[0] = 12.34f; s.w_dps[1] = -250.0f; s.w_dps[2] = 0.05f;
    s.a_ms2[0] = 0.12f; s.a_ms2[1] = -9.81f; s.a_ms2[2] = 150.0f;
    s.bias_dps[0] = 0.25f; s.bias_dps[1] = -1.5f; s.bias_dps[2] = 0.0f;
    s.fin[0] = 1.0f; s.fin[1] = -1.0f; s.fin[2] = 0.5f; s.fin[3] = -0.25f;
    s.loop_us = 1160u; s.faults = 0x0081u;
    return s;
}

static void test_state_roundtrip_within_quantisation(void)
{
    const LogState in = sample();
    uint8_t r[LOGREC_SIZE];
    LogRec_PackState(r, &in);
    TEST_ASSERT_TRUE(LogRec_Valid(r));
    LogState out;
    TEST_ASSERT_TRUE(LogRec_UnpackState(r, &out));
    TEST_ASSERT_EQUAL_UINT32(in.t_us, out.t_us);
    TEST_ASSERT_EQUAL_UINT16(in.seq, out.seq);
    TEST_ASSERT_EQUAL_UINT8(in.phase, out.phase);
    TEST_ASSERT_EQUAL_UINT8(in.mode, out.mode);
    TEST_ASSERT_EQUAL_UINT8(in.flags, out.flags);
    for (int i = 0; i < 4; i++) { TEST_ASSERT_FLOAT_WITHIN(1.0f/32767.0f, in.q[i], out.q[i]); }
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.051f,  in.w_dps[i],    out.w_dps[i]);
        TEST_ASSERT_FLOAT_WITHIN(0.0051f, in.a_ms2[i],    out.a_ms2[i]);
        TEST_ASSERT_FLOAT_WITHIN(0.0051f, in.bias_dps[i], out.bias_dps[i]);
    }
    for (int i = 0; i < 4; i++) { TEST_ASSERT_FLOAT_WITHIN(0.5f/127.0f, in.fin[i], out.fin[i]); }
    TEST_ASSERT_EQUAL_UINT16(in.loop_us, out.loop_us);
    TEST_ASSERT_EQUAL_UINT16(in.faults, out.faults);
}

static void test_every_single_bit_flip_detected(void)
{
    const LogState in = sample();
    uint8_t r[LOGREC_SIZE];
    LogRec_PackState(r, &in);
    for (uint32_t bit = 0u; bit < LOGREC_SIZE * 8u; bit++) {
        uint8_t c[LOGREC_SIZE];
        memcpy(c, r, sizeof c);
        c[bit / 8u] ^= (uint8_t)(1u << (bit % 8u));
        TEST_ASSERT_FALSE(LogRec_Valid(c));
    }
}

static void test_erased_flash_is_not_a_record(void)
{
    uint8_t r[LOGREC_SIZE];
    memset(r, 0xFF, sizeof r);
    TEST_ASSERT_FALSE(LogRec_Valid(r));
}

static void test_saturation_and_nan(void)
{
    LogState in = sample();
    in.w_dps[0] = 1.0e6f;        /* beyond +/-3276.7 dps */
    in.a_ms2[1] = -1.0e6f;
    in.q[1] = NAN;
    in.fin[0] = 5.0f;
    uint8_t r[LOGREC_SIZE];
    LogRec_PackState(r, &in);
    LogState out;
    TEST_ASSERT_TRUE(LogRec_UnpackState(r, &out));
    TEST_ASSERT_FLOAT_WITHIN(0.01f,  3276.7f, out.w_dps[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -327.67f, out.a_ms2[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -32768.0f / 32767.0f, out.q[1]);   /* INT16_MIN sentinel */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, out.fin[0]);
}

static void test_event_and_boot_records(void)
{
    uint8_t r[LOGREC_SIZE];
    LogEvent e;
    memset(&e, 0, sizeof e);
    e.code = 7u; e.phase = 8u; e.arg0 = 0xDEADBEEFu; e.arg1 = 1u;
    memcpy(e.text, "fault imu_read", 14);
    LogRec_PackEvent(r, &e);
    TEST_ASSERT_TRUE(LogRec_Valid(r));
    TEST_ASSERT_EQUAL_HEX8(LOGREC_EVENT, r[0]);
    TEST_ASSERT_EQUAL_MEMORY("fault imu_read", &r[20], 14);

    LogBoot b;
    memset(&b, 0, sizeof b);
    b.frame_hz = 100u; b.log_hz = 50u;
    memcpy(b.build, "abc1234+ Nov  3 2026", 20);
    LogRec_PackBoot(r, &b);
    TEST_ASSERT_TRUE(LogRec_Valid(r));
    TEST_ASSERT_EQUAL_HEX8(LOGREC_BOOT, r[0]);
    TEST_ASSERT_EQUAL_UINT8(LOGREC_VERSION, r[1]);
    LogState s;
    TEST_ASSERT_FALSE(LogRec_UnpackState(r, &s));          /* wrong type */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_state_roundtrip_within_quantisation);
    RUN_TEST(test_every_single_bit_flip_detected);
    RUN_TEST(test_erased_flash_is_not_a_record);
    RUN_TEST(test_saturation_and_nan);
    RUN_TEST(test_event_and_boot_records);
    return UNITY_END();
}
