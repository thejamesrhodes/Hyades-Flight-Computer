#include "unity.h"
#include "nmea.h"
#include <stdio.h>
#include <string.h>

static NmeaParser P;
static GpsData G;

void setUp(void) { Nmea_Init(&P, &G); }
void tearDown(void) {}

/* Feed "$<body>*<checksum>\r\n" with a correct (or deliberately wrong) checksum. */
static void feed_sentence(const char *body, bool corrupt, uint32_t t)
{
    uint8_t ck = 0u;
    for (const char *c = body; *c != '\0'; c++) { ck ^= (uint8_t)*c; }
    if (corrupt) { ck ^= 0x01u; }
    char buf[128];
    (void)snprintf(buf, sizeof buf, "$%s*%02X\r\n", body, (unsigned)ck);
    for (const char *c = buf; *c != '\0'; c++) { Nmea_Feed(&P, &G, *c, t); }
}

static void test_parse_number(void)
{
    TEST_ASSERT_EQUAL_DOUBLE(545.4, Nmea_ParseNum("545.4"));
    TEST_ASSERT_EQUAL_DOUBLE(-12.25, Nmea_ParseNum("-12.25"));
    TEST_ASSERT_EQUAL_DOUBLE(7.0, Nmea_ParseNum("7"));
    TEST_ASSERT_EQUAL_DOUBLE(0.0, Nmea_ParseNum(""));
}

static void test_gga_fix(void)
{
    feed_sentence("GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,", false, 1000u);
    TEST_ASSERT_EQUAL_UINT32(1u, G.gga);
    TEST_ASSERT_EQUAL_UINT8(1u, G.fix_quality);
    TEST_ASSERT_EQUAL_UINT8(8u, G.sats);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.9f, G.hdop);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 545.4f, G.alt_msl_m);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 48.1173, G.lat_deg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 11.516666667, G.lon_deg);
    TEST_ASSERT_EQUAL_UINT32(123519u, G.utc_hhmmss);
    TEST_ASSERT_TRUE(Nmea_HasFix(&G, 1500u, 2500u));
    TEST_ASSERT_FALSE(Nmea_HasFix(&G, 4000u, 2500u));     /* stale */
}

static void test_southern_western_hemisphere(void)
{
    feed_sentence("GPGGA,000001,3352.000,S,15112.000,W,1,05,1.2,10.0,M,0,M,,", false, 0u);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -33.866666667, G.lat_deg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -151.2, G.lon_deg);
}

static void test_no_fix_keeps_last_position(void)
{
    feed_sentence("GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,", false, 0u);
    feed_sentence("GPGGA,,,,,,0,00,99.99,,,,,,", false, 10u);
    TEST_ASSERT_EQUAL_UINT8(0u, G.fix_quality);
    TEST_ASSERT_FALSE(Nmea_HasFix(&G, 20u, 2500u));
    TEST_ASSERT_TRUE(Nmea_Alive(&G, 20u, 2500u));
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 545.4f, G.alt_msl_m);
}

static void test_rmc(void)
{
    feed_sentence("GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W", false, 0u);
    TEST_ASSERT_TRUE(G.rmc_valid);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 22.4f * 0.514444f, G.speed_mps);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 84.4f, G.course_deg);
}

static void test_bad_checksum_rejected(void)
{
    feed_sentence("GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,", true, 0u);
    TEST_ASSERT_EQUAL_UINT32(0u, G.sentences);
    TEST_ASSERT_EQUAL_UINT32(1u, G.cksum_err);
    TEST_ASSERT_FALSE(Nmea_Alive(&G, 0u, 2500u));
}

static void test_overlong_line_dropped_then_recovers(void)
{
    Nmea_Feed(&P, &G, '$', 0u);
    for (uint32_t i = 0u; i < 200u; i++) { Nmea_Feed(&P, &G, 'A', 0u); }
    TEST_ASSERT_EQUAL_UINT32(1u, G.cksum_err);
    feed_sentence("GNGGA,1,4807.038,N,01131.000,E,1,08,0.9,5.0,M,0,M,,", false, 0u);
    TEST_ASSERT_EQUAL_UINT32(1u, G.gga);
}

static void test_other_sentences_counted_not_parsed(void)
{
    feed_sentence("GPGSV,3,1,11,03,03,111,00", false, 0u);
    TEST_ASSERT_EQUAL_UINT32(1u, G.sentences);
    TEST_ASSERT_EQUAL_UINT32(0u, G.gga + G.rmc);
}

static void test_alive_handles_tick_wrap(void)
{
    feed_sentence("GPGSV,3,1,11", false, 0xFFFFFF00u);
    TEST_ASSERT_TRUE(Nmea_Alive(&G, 0x00000100u, 2500u));   /* 512 ms later, wrapped */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_parse_number);
    RUN_TEST(test_gga_fix);
    RUN_TEST(test_southern_western_hemisphere);
    RUN_TEST(test_no_fix_keeps_last_position);
    RUN_TEST(test_rmc);
    RUN_TEST(test_bad_checksum_rejected);
    RUN_TEST(test_overlong_line_dropped_then_recovers);
    RUN_TEST(test_other_sentences_counted_not_parsed);
    RUN_TEST(test_alive_handles_tick_wrap);
    return UNITY_END();
}
