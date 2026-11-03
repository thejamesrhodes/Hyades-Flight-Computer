#include "unity.h"
#include "crc.h"
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

static const uint8_t k_check[] = "123456789";   /* standard check string */

static void test_crc16_check_value(void)
{
    TEST_ASSERT_EQUAL_HEX16(0x29B1u, crc16_ccitt(k_check, 9u));   /* CCITT-FALSE */
}

static void test_crc32_check_value(void)
{
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, crc32_ieee(k_check, 9u)); /* zlib.crc32 */
}

static void test_crc32_chains(void)
{
    const uint32_t c = crc32_ieee_update(crc32_ieee(k_check, 4u), &k_check[4], 5u);
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926u, c);
}

static void test_crc_empty(void)
{
    TEST_ASSERT_EQUAL_HEX16(0xFFFFu, crc16_ccitt(k_check, 0u));
    TEST_ASSERT_EQUAL_HEX32(0x00000000u, crc32_ieee(k_check, 0u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc16_check_value);
    RUN_TEST(test_crc32_check_value);
    RUN_TEST(test_crc32_chains);
    RUN_TEST(test_crc_empty);
    return UNITY_END();
}
