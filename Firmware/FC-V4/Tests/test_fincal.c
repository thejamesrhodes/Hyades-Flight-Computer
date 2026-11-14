/* CAL-01: canard zero calibration record and journal scan. */
#include "unity.h"
#include "fincal.h"
#include "crc.h"
#include <string.h>

#define SLOTS 16u
static uint8_t J[SLOTS * FINCAL_REC_SIZE];

void setUp(void) { memset(J, 0xFF, sizeof J); }
void tearDown(void) {}

static void put(uint32_t slot, uint16_t seq, int32_t a, int32_t b, int32_t c, int32_t d)
{
    const int32_t off[4] = { a, b, c, d };
    FinCal_Pack(&J[slot * FINCAL_REC_SIZE], seq, off);
}

static void test_pack_unpack_round_trip(void)
{
    put(0, 7, 120, -40, 0, -FINCAL_LIMIT_MDEG);
    FinCal c;
    TEST_ASSERT_TRUE(FinCal_Unpack(J, &c));
    TEST_ASSERT_TRUE(c.valid);
    TEST_ASSERT_EQUAL_UINT8(FINCAL_VERSION, c.version);
    TEST_ASSERT_EQUAL_UINT16(7, c.seq);
    TEST_ASSERT_EQUAL_INT32(120, c.off_mdeg[0]);
    TEST_ASSERT_EQUAL_INT32(-40, c.off_mdeg[1]);
    TEST_ASSERT_EQUAL_INT32(-65000, c.off_mdeg[3]);
    TEST_ASSERT_TRUE(FinCal_Within(c.off_mdeg, FINCAL_LIMIT_MDEG));
    TEST_ASSERT_FALSE(FinCal_Within(c.off_mdeg, 35000));     /* beyond the full-authority zero */
}

/* every single-bit flip in the record is rejected (CRC, magic, version) */
static void test_any_bit_flip_rejected(void)
{
    put(0, 3, 500, 600, -700, 800);
    for (uint32_t bit = 0u; bit < (FINCAL_REC_SIZE * 8u); bit++) {
        uint8_t r[FINCAL_REC_SIZE];
        memcpy(r, J, sizeof r);
        r[bit / 8u] ^= (uint8_t)(1u << (bit % 8u));
        FinCal c;
        TEST_ASSERT_FALSE(FinCal_Unpack(r, &c));
        TEST_ASSERT_FALSE(c.valid);
    }
}

/* a well-formed record outside +/-65 deg is not trusted */
static void test_out_of_range_rejected(void)
{
    put(0, 1, FINCAL_LIMIT_MDEG + 1, 0, 0, 0);
    FinCal c;
    TEST_ASSERT_FALSE(FinCal_Unpack(J, &c));
}

static void put_v2(uint32_t slot, uint16_t seq, int32_t a, int32_t b, int32_t c, int32_t d)
{
    uint8_t *r = &J[slot * FINCAL_REC_SIZE];
    put(slot, seq, 0, 0, 0, 0);
    const int32_t off[4] = { a, b, c, d };
    for (uint32_t i = 0u; i < 4u; i++) { for (uint32_t k = 0u; k < 4u; k++) { r[8u + (4u * i) + k] = (uint8_t)((uint32_t)off[i] >> (8u * k)); } }
    r[6] = (uint8_t)FINCAL_VERSION_OLD;
    const uint32_t crc = crc32_ieee(r, 28u);
    for (uint32_t k = 0u; k < 4u; k++) { r[28u + k] = (uint8_t)(crc >> (8u * k)); }
}

/* a version-2 record holds zeros aligned at the old 6.667 us/deg scale: it is
 * read and converted x 3/4 (the same pulse), so the alignment survives the
 * scale correction (REVIEW-01 F1) */
static void test_old_version_converted(void)
{
    put_v2(0, 5, 100, -24000, 86000, -1);
    FinCal c;
    TEST_ASSERT_TRUE(FinCal_Unpack(J, &c));
    TEST_ASSERT_EQUAL_UINT8(FINCAL_VERSION_OLD, c.version);
    TEST_ASSERT_EQUAL_UINT16(5, c.seq);
    TEST_ASSERT_EQUAL_INT32(75, c.off_mdeg[0]);
    TEST_ASSERT_EQUAL_INT32(-18000, c.off_mdeg[1]);
    TEST_ASSERT_EQUAL_INT32(64500, c.off_mdeg[2]);
    TEST_ASSERT_EQUAL_INT32(-1, c.off_mdeg[3]);              /* -0.75 rounds to -1 */
    /* same pulse: mdeg x 1200 us / range, old 180 deg against new 135 deg */
    TEST_ASSERT_EQUAL_INT32(-24000 * 1200 / 180, c.off_mdeg[1] * 1200 / 135);
    /* a newer save wins and reads back as the current version */
    put(1, 6, 10, 20, 30, 40);
    FinCalScan s;
    FinCal_Scan(J, sizeof J, &s);
    TEST_ASSERT_TRUE(s.cal.valid);
    TEST_ASSERT_EQUAL_UINT8(FINCAL_VERSION, s.cal.version);
    TEST_ASSERT_EQUAL_INT32(10, s.cal.off_mdeg[0]);
    TEST_ASSERT_EQUAL_UINT32(0, s.bad);
}

/* an old zero that converts to beyond +/-65 deg, or is outside the old +/-90
 * deg, is not trusted; an unknown version is not read */
static void test_old_version_out_of_range_rejected(void)
{
    FinCal c;
    put_v2(0, 1, 90000, 0, 0, 0);                            /* 67.5 deg at the new scale */
    TEST_ASSERT_FALSE(FinCal_Unpack(J, &c));
    put_v2(0, 1, 0, 0, -90001, 0);
    TEST_ASSERT_FALSE(FinCal_Unpack(J, &c));
    put_v2(0, 1, 0, 0, 0, 2000000000);                       /* would overflow x 3: refused first */
    TEST_ASSERT_FALSE(FinCal_Unpack(J, &c));
    put_v2(0, 1, 1, 2, 3, 4);
    J[6] = 1u;                                               /* version 1: CRC now wrong as well */
    TEST_ASSERT_FALSE(FinCal_Unpack(J, &c));
}

static void test_empty_journal(void)
{
    FinCalScan s;
    FinCal_Scan(J, sizeof J, &s);
    TEST_ASSERT_FALSE(s.cal.valid);
    TEST_ASSERT_EQUAL_UINT32(0, s.next);
    TEST_ASSERT_EQUAL_UINT32(SLOTS, s.slots);
    TEST_ASSERT_EQUAL_UINT32(0, s.bad);
}

/* newest valid record wins; a torn last record is skipped, not trusted, and
 * the next save goes after it */
static void test_newest_wins_torn_skipped(void)
{
    put(0, 1, 100, 0, 0, 0);
    put(1, 2, 200, 0, 0, 0);
    put(2, 3, 300, 0, 0, 0);
    memset(&J[2 * FINCAL_REC_SIZE + 12], 0xFF, 20);          /* power lost mid-write */
    FinCalScan s;
    FinCal_Scan(J, sizeof J, &s);
    TEST_ASSERT_TRUE(s.cal.valid);
    TEST_ASSERT_EQUAL_UINT16(2, s.cal.seq);
    TEST_ASSERT_EQUAL_INT32(200, s.cal.off_mdeg[0]);
    TEST_ASSERT_EQUAL_UINT32(3, s.next);
    TEST_ASSERT_EQUAL_UINT32(1, s.bad);
}

/* foreign data (an old flight log where the journal now lives) is never read
 * as a calibration and leaves no free slot: the journal reports full */
static void test_foreign_data_is_full_and_invalid(void)
{
    for (uint32_t i = 0u; i < sizeof J; i++) { J[i] = (uint8_t)(i * 37u + 11u); }
    FinCalScan s;
    FinCal_Scan(J, sizeof J, &s);
    TEST_ASSERT_FALSE(s.cal.valid);
    TEST_ASSERT_EQUAL_UINT32(SLOTS, s.next);
    TEST_ASSERT_EQUAL_UINT32(SLOTS, s.bad);
}

/* free space is the erased tail: an erased hole before a later record is
 * history, not a place to write */
static void test_next_is_after_last_written(void)
{
    put(0, 1, 1, 1, 1, 1);
    put(3, 2, 2, 2, 2, 2);
    FinCalScan s;
    FinCal_Scan(J, sizeof J, &s);
    TEST_ASSERT_EQUAL_UINT32(4, s.next);
    TEST_ASSERT_EQUAL_UINT16(2, s.cal.seq);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_pack_unpack_round_trip);
    RUN_TEST(test_any_bit_flip_rejected);
    RUN_TEST(test_out_of_range_rejected);
    RUN_TEST(test_old_version_converted);
    RUN_TEST(test_old_version_out_of_range_rejected);
    RUN_TEST(test_empty_journal);
    RUN_TEST(test_newest_wins_torn_skipped);
    RUN_TEST(test_foreign_data_is_full_and_invalid);
    RUN_TEST(test_next_is_after_last_written);
    return UNITY_END();
}
