/*
 * Ring buffer unit tests. Built against SM_PORT_NONE: single-threaded and
 * deterministic, so a failure here is a logic bug and never a race.
 */
#include "test_harness.h"

#include "sm_ringbuf.h"

#define CAP 8

static sm_rb_t  rb;
static uint32_t store[CAP];

static void setup(void)
{
    memset(store, 0, sizeof(store));
    CHECK_EQ(sm_rb_init(&rb, store, sizeof(uint32_t), CAP), 0);
}

TEST(init_rejects_bad_arguments)
{
    sm_rb_t  bad;
    uint32_t s[4];

    CHECK_EQ(sm_rb_init(&bad, NULL, 4, 4), -1);
    CHECK_EQ(sm_rb_init(&bad, s, 0, 4), -1);
    CHECK_EQ(sm_rb_init(&bad, s, 4, 0), -1);
    CHECK_EQ(sm_rb_init(NULL, s, 4, 4), -1);
}

TEST(empty_buffer_yields_nothing)
{
    uint32_t out[CAP];

    setup();
    CHECK_EQ(sm_rb_count(&rb), 0);
    CHECK_EQ(sm_rb_read(&rb, out, CAP), 0);
    CHECK_EQ(sm_rb_peek(&rb, out, CAP), 0);
    CHECK_EQ(sm_rb_consume(&rb, 5), 0);
}

TEST(fifo_order_is_preserved)
{
    uint32_t out[CAP];
    uint32_t i;

    setup();
    for (i = 0; i < 5; i++) {
        CHECK_EQ(sm_rb_push(&rb, &i), false);
    }
    CHECK_EQ(sm_rb_count(&rb), 5);
    CHECK_EQ(sm_rb_read(&rb, out, CAP), 5);
    for (i = 0; i < 5; i++) {
        CHECK_EQ(out[i], i); /* oldest first */
    }
    CHECK_EQ(sm_rb_count(&rb), 0);
}

TEST(fills_to_capacity_without_dropping)
{
    sm_rb_stats_t st;
    uint32_t      i;

    setup();
    for (i = 0; i < CAP; i++) {
        CHECK_EQ(sm_rb_push(&rb, &i), false); /* nothing evicted yet */
    }
    sm_rb_stats(&rb, &st);
    CHECK_EQ(st.count, CAP);
    CHECK_EQ(st.pushed, CAP);
    CHECK_EQ(st.dropped, 0);
}

/*
 * The core requirement: on overflow the buffer must retain the LATEST N,
 * not the first N, and must say how many it lost.
 */
TEST(overflow_retains_latest_and_counts_drops)
{
    uint32_t      out[CAP];
    sm_rb_stats_t st;
    uint32_t      i;

    setup();
    for (i = 0; i < CAP + 5; i++) {
        bool evicted = sm_rb_push(&rb, &i);
        CHECK_EQ(evicted, i >= CAP); /* eviction starts exactly at capacity */
    }

    sm_rb_stats(&rb, &st);
    CHECK_EQ(st.count, CAP);
    CHECK_EQ(st.pushed, CAP + 5);
    CHECK_EQ(st.dropped, 5);

    CHECK_EQ(sm_rb_read(&rb, out, CAP), CAP);
    for (i = 0; i < CAP; i++) {
        CHECK_EQ(out[i], i + 5); /* 0..4 were evicted; 5..12 survive */
    }
}

/* Wraparound is where ring buffers die. Push far past capacity, interleaved
 * with reads, and check the contents every time. */
TEST(wraparound_stays_correct_over_many_cycles)
{
    uint32_t out[CAP];
    uint32_t v, round;

    setup();
    v = 0;
    for (round = 0; round < 50; round++) {
        uint32_t n = (round % 3) + 1; /* uneven push/read sizes */
        uint32_t i;

        for (i = 0; i < n; i++, v++) {
            sm_rb_push(&rb, &v);
        }
        CHECK_EQ(sm_rb_read(&rb, out, n), n);
        for (i = 0; i < n; i++) {
            CHECK_EQ(out[i], v - n + i);
        }
        CHECK_EQ(sm_rb_count(&rb), 0);
    }
}

TEST(partial_read_leaves_remainder_in_order)
{
    uint32_t out[CAP];
    uint32_t i;

    setup();
    for (i = 0; i < 6; i++) {
        sm_rb_push(&rb, &i);
    }
    CHECK_EQ(sm_rb_read(&rb, out, 2), 2);
    CHECK_EQ(out[0], 0);
    CHECK_EQ(out[1], 1);
    CHECK_EQ(sm_rb_count(&rb), 4);

    CHECK_EQ(sm_rb_read(&rb, out, CAP), 4);
    CHECK_EQ(out[0], 2);
    CHECK_EQ(out[3], 5);
}

TEST(read_of_more_than_available_returns_what_exists)
{
    uint32_t out[CAP * 2];
    uint32_t i;

    setup();
    for (i = 0; i < 3; i++) {
        sm_rb_push(&rb, &i);
    }
    CHECK_EQ(sm_rb_read(&rb, out, CAP * 2), 3);
}

TEST(peek_does_not_consume)
{
    uint32_t out[CAP];
    uint32_t i;

    setup();
    for (i = 0; i < 4; i++) {
        sm_rb_push(&rb, &i);
    }

    CHECK_EQ(sm_rb_peek(&rb, out, CAP), 4);
    CHECK_EQ(sm_rb_count(&rb), 4); /* still there */
    CHECK_EQ(sm_rb_peek(&rb, out, CAP), 4);
    CHECK_EQ(out[0], 0);

    /* peek-then-release is the retryable-notification path */
    CHECK_EQ(sm_rb_consume(&rb, 2), 2);
    CHECK_EQ(sm_rb_count(&rb), 2);
    CHECK_EQ(sm_rb_peek(&rb, out, CAP), 2);
    CHECK_EQ(out[0], 2);
}

TEST(consume_clamps_to_available)
{
    uint32_t i;

    setup();
    for (i = 0; i < 3; i++) {
        sm_rb_push(&rb, &i);
    }
    CHECK_EQ(sm_rb_consume(&rb, 99), 3);
    CHECK_EQ(sm_rb_count(&rb), 0);
}

TEST(reset_clears_records_but_keeps_lifetime_counters)
{
    sm_rb_stats_t st;
    uint32_t      i;

    setup();
    for (i = 0; i < CAP + 2; i++) {
        sm_rb_push(&rb, &i);
    }
    sm_rb_reset(&rb);

    sm_rb_stats(&rb, &st);
    CHECK_EQ(st.count, 0);
    CHECK_EQ(st.pushed, CAP + 2); /* diagnostics survive a reset */
    CHECK_EQ(st.dropped, 2);
}

/* A capacity-1 buffer is the degenerate case that off-by-one bugs love. */
TEST(capacity_one_behaves)
{
    sm_rb_t  one;
    uint32_t s[1];
    uint32_t out[1];
    uint32_t v;

    CHECK_EQ(sm_rb_init(&one, s, sizeof(uint32_t), 1), 0);
    v = 11;
    CHECK_EQ(sm_rb_push(&one, &v), false);
    v = 22;
    CHECK_EQ(sm_rb_push(&one, &v), true);
    CHECK_EQ(sm_rb_count(&one), 1);
    CHECK_EQ(sm_rb_read(&one, out, 1), 1);
    CHECK_EQ(out[0], 22);
    sm_rb_deinit(&one);
}

TEST(non_power_of_two_capacity_is_supported)
{
    /* The product capacities are 100 and 20. Neither is a power of two, so
     * the implementation must not depend on mask-based wrapping. */
    sm_rb_t odd;
    uint8_t s[100];
    uint8_t out[100];
    int     i;

    CHECK_EQ(sm_rb_init(&odd, s, 1, 100), 0);
    for (i = 0; i < 250; i++) {
        uint8_t v = (uint8_t)i;
        sm_rb_push(&odd, &v);
    }
    CHECK_EQ(sm_rb_count(&odd), 100);
    CHECK_EQ(sm_rb_read(&odd, out, 100), 100);
    for (i = 0; i < 100; i++) {
        CHECK_EQ(out[i], (uint8_t)(150 + i)); /* latest 100 of 0..249 */
    }
    sm_rb_deinit(&odd);
}

TEST_MAIN_BEGIN("sm_ringbuf")
RUN(init_rejects_bad_arguments);
RUN(empty_buffer_yields_nothing);
RUN(fifo_order_is_preserved);
RUN(fills_to_capacity_without_dropping);
RUN(overflow_retains_latest_and_counts_drops);
RUN(wraparound_stays_correct_over_many_cycles);
RUN(partial_read_leaves_remainder_in_order);
RUN(read_of_more_than_available_returns_what_exists);
RUN(peek_does_not_consume);
RUN(consume_clamps_to_available);
RUN(reset_clears_records_but_keeps_lifetime_counters);
RUN(capacity_one_behaves);
RUN(non_power_of_two_capacity_is_supported);
TEST_MAIN_END()
