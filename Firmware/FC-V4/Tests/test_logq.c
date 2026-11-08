#include "unity.h"
#include "logq.h"
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

static uint8_t B[8][LOGREC_SIZE];
static void rec(uint8_t *r, uint8_t tag) { memset(r, 0, LOGREC_SIZE); r[0] = tag; }

static void test_armed_keeps_latest_cap_records_in_order(void)
{
    LogQ q; LogQ_Init(&q, B, 8u);
    uint8_t r[LOGREC_SIZE];
    for (uint8_t i = 1u; i <= 20u; i++) { rec(r, i); TEST_ASSERT_TRUE(LogQ_Push(&q, r)); }
    TEST_ASSERT_EQUAL_UINT16(8u, LogQ_Count(&q));
    for (uint8_t i = 13u; i <= 20u; i++) { TEST_ASSERT_EQUAL_UINT8(i, LogQ_Peek(&q)[0]); LogQ_Pop(&q); }
    TEST_ASSERT_NULL(LogQ_Peek(&q));
    TEST_ASSERT_EQUAL_UINT32(0u, q.dropped);
}

static void test_committed_never_overwrites_backlog(void)
{
    LogQ q; LogQ_Init(&q, B, 8u);
    uint8_t r[LOGREC_SIZE];
    for (uint8_t i = 1u; i <= 8u; i++) { rec(r, i); (void)LogQ_Push(&q, r); }
    LogQ_Commit(&q);
    rec(r, 99u);
    TEST_ASSERT_FALSE(LogQ_Push(&q, r));                 /* full: new record dropped */
    TEST_ASSERT_EQUAL_UINT32(1u, q.dropped);
    TEST_ASSERT_EQUAL_UINT8(1u, LogQ_Peek(&q)[0]);       /* oldest pre-trigger kept */
}

/* The flight drain: 1 record pushed every 2nd frame, 2 drained per frame.
 * A full ring must clear, with no loss, in well under a second of frames. */
static void test_drain_outpaces_production_no_loss(void)
{
    static uint8_t big[256][LOGREC_SIZE];
    LogQ q; LogQ_Init(&q, big, 256u);
    uint8_t r[LOGREC_SIZE];
    uint32_t n_in = 0u, n_out = 0u, frames = 0u;
    for (uint32_t i = 0u; i < 1000u; i++) { rec(r, (uint8_t)i); (void)LogQ_Push(&q, r); n_in++; }
    LogQ_Commit(&q);
    n_in = 256u;                                         /* what the ring holds */
    uint8_t expect = (uint8_t)(1000u - 256u);
    while ((LogQ_Count(&q) > 1u) && (frames < 2000u)) {
        /* firmware order: drain first, then this frame's record */
        for (uint32_t k = 0u; (k < 2u) && (LogQ_Peek(&q) != NULL); k++) {
            const uint8_t *p = LogQ_Peek(&q);
            if (n_out < 256u) { TEST_ASSERT_EQUAL_UINT8(expect, p[0]); expect++; }
            LogQ_Pop(&q); n_out++;
        }
        if ((frames % 2u) == 0u) { rec(r, 0xEEu); TEST_ASSERT_TRUE(LogQ_Push(&q, r)); n_in++; }
        frames++;
    }
    TEST_ASSERT_EQUAL_UINT32(0u, q.dropped);
    TEST_ASSERT_LESS_THAN_UINT32(200u, frames);          /* 1 s at 200 Hz */
}

static void test_reset_rearms_and_clears(void)
{
    LogQ q; LogQ_Init(&q, B, 8u);
    uint8_t r[LOGREC_SIZE]; rec(r, 5u);
    (void)LogQ_Push(&q, r); LogQ_Commit(&q); LogQ_Reset(&q);
    TEST_ASSERT_FALSE(q.committed);
    TEST_ASSERT_EQUAL_UINT16(0u, LogQ_Count(&q));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_armed_keeps_latest_cap_records_in_order);
    RUN_TEST(test_committed_never_overwrites_backlog);
    RUN_TEST(test_drain_outpaces_production_no_loss);
    RUN_TEST(test_reset_rearms_and_clears);
    return UNITY_END();
}
