/*
 * Ring buffer unit tests, ported from the custom harness to Ztest.
 *
 * The core is built here against SM_PORT_ZEPHYR (the live firmware port), but
 * these cases stay single-threaded and deterministic, so a failure is a logic
 * bug and never a race. Assertion intent is unchanged from tests/test_ringbuf.c;
 * only the harness (TEST/CHECK -> ZTEST/zassert) is different.
 */
#include <string.h>

#include <zephyr/ztest.h>

#include "sm_ringbuf.h"

#define CAP 8

static sm_rb_t  rb;
static uint32_t store[CAP];

static void setup(void)
{
    memset(store, 0, sizeof(store));
    zassert_equal(sm_rb_init(&rb, store, sizeof(uint32_t), CAP), 0);
}

ZTEST(ringbuf, init_rejects_bad_arguments)
{
    sm_rb_t  bad;
    uint32_t s[4];

    zassert_equal(sm_rb_init(&bad, NULL, 4, 4), -1);
    zassert_equal(sm_rb_init(&bad, s, 0, 4), -1);
    zassert_equal(sm_rb_init(&bad, s, 4, 0), -1);
    zassert_equal(sm_rb_init(NULL, s, 4, 4), -1);
}

ZTEST(ringbuf, empty_buffer_yields_nothing)
{
    uint32_t out[CAP];

    setup();
    zassert_equal(sm_rb_count(&rb), 0);
    zassert_equal(sm_rb_read(&rb, out, CAP), 0);
    zassert_equal(sm_rb_peek(&rb, out, CAP), 0);
    zassert_equal(sm_rb_consume(&rb, 5), 0);
}

ZTEST(ringbuf, fifo_order_is_preserved)
{
    uint32_t out[CAP];
    uint32_t i;

    setup();
    for (i = 0; i < 5; i++) {
        zassert_equal(sm_rb_push(&rb, &i), false);
    }
    zassert_equal(sm_rb_count(&rb), 5);
    zassert_equal(sm_rb_read(&rb, out, CAP), 5);
    for (i = 0; i < 5; i++) {
        zassert_equal(out[i], i); /* oldest first */
    }
    zassert_equal(sm_rb_count(&rb), 0);
}

ZTEST(ringbuf, fills_to_capacity_without_dropping)
{
    sm_rb_stats_t st;
    uint32_t      i;

    setup();
    for (i = 0; i < CAP; i++) {
        zassert_equal(sm_rb_push(&rb, &i), false); /* nothing evicted yet */
    }
    sm_rb_stats(&rb, &st);
    zassert_equal(st.count, CAP);
    zassert_equal(st.pushed, CAP);
    zassert_equal(st.dropped, 0);
}

/*
 * The core requirement: on overflow the buffer must retain the LATEST N,
 * not the first N, and must say how many it lost.
 */
ZTEST(ringbuf, overflow_retains_latest_and_counts_drops)
{
    uint32_t      out[CAP];
    sm_rb_stats_t st;
    uint32_t      i;

    setup();
    for (i = 0; i < CAP + 5; i++) {
        bool evicted = sm_rb_push(&rb, &i);
        zassert_equal(evicted, i >= CAP); /* eviction starts exactly at capacity */
    }

    sm_rb_stats(&rb, &st);
    zassert_equal(st.count, CAP);
    zassert_equal(st.pushed, CAP + 5);
    zassert_equal(st.dropped, 5);

    zassert_equal(sm_rb_read(&rb, out, CAP), CAP);
    for (i = 0; i < CAP; i++) {
        zassert_equal(out[i], i + 5); /* 0..4 were evicted; 5..12 survive */
    }
}

/* Wraparound is where ring buffers die. Push far past capacity, interleaved
 * with reads, and check the contents every time. */
ZTEST(ringbuf, wraparound_stays_correct_over_many_cycles)
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
        zassert_equal(sm_rb_read(&rb, out, n), n);
        for (i = 0; i < n; i++) {
            zassert_equal(out[i], v - n + i);
        }
        zassert_equal(sm_rb_count(&rb), 0);
    }
}

ZTEST(ringbuf, partial_read_leaves_remainder_in_order)
{
    uint32_t out[CAP];
    uint32_t i;

    setup();
    for (i = 0; i < 6; i++) {
        sm_rb_push(&rb, &i);
    }
    zassert_equal(sm_rb_read(&rb, out, 2), 2);
    zassert_equal(out[0], 0);
    zassert_equal(out[1], 1);
    zassert_equal(sm_rb_count(&rb), 4);

    zassert_equal(sm_rb_read(&rb, out, CAP), 4);
    zassert_equal(out[0], 2);
    zassert_equal(out[3], 5);
}

ZTEST(ringbuf, read_of_more_than_available_returns_what_exists)
{
    uint32_t out[CAP * 2];
    uint32_t i;

    setup();
    for (i = 0; i < 3; i++) {
        sm_rb_push(&rb, &i);
    }
    zassert_equal(sm_rb_read(&rb, out, CAP * 2), 3);
}

ZTEST(ringbuf, peek_does_not_consume)
{
    uint32_t out[CAP];
    uint32_t i;

    setup();
    for (i = 0; i < 4; i++) {
        sm_rb_push(&rb, &i);
    }

    zassert_equal(sm_rb_peek(&rb, out, CAP), 4);
    zassert_equal(sm_rb_count(&rb), 4); /* still there */
    zassert_equal(sm_rb_peek(&rb, out, CAP), 4);
    zassert_equal(out[0], 0);

    /* peek-then-release is the retryable-notification path */
    zassert_equal(sm_rb_consume(&rb, 2), 2);
    zassert_equal(sm_rb_count(&rb), 2);
    zassert_equal(sm_rb_peek(&rb, out, CAP), 2);
    zassert_equal(out[0], 2);
}

ZTEST(ringbuf, consume_clamps_to_available)
{
    uint32_t i;

    setup();
    for (i = 0; i < 3; i++) {
        sm_rb_push(&rb, &i);
    }
    zassert_equal(sm_rb_consume(&rb, 99), 3);
    zassert_equal(sm_rb_count(&rb), 0);
}

ZTEST(ringbuf, reset_clears_records_but_keeps_lifetime_counters)
{
    sm_rb_stats_t st;
    uint32_t      i;

    setup();
    for (i = 0; i < CAP + 2; i++) {
        sm_rb_push(&rb, &i);
    }
    sm_rb_reset(&rb);

    sm_rb_stats(&rb, &st);
    zassert_equal(st.count, 0);
    zassert_equal(st.pushed, CAP + 2); /* diagnostics survive a reset */
    zassert_equal(st.dropped, 2);
}

/* A capacity-1 buffer is the degenerate case that off-by-one bugs love. */
ZTEST(ringbuf, capacity_one_behaves)
{
    sm_rb_t  one;
    uint32_t s[1];
    uint32_t out[1];
    uint32_t v;

    zassert_equal(sm_rb_init(&one, s, sizeof(uint32_t), 1), 0);
    v = 11;
    zassert_equal(sm_rb_push(&one, &v), false);
    v = 22;
    zassert_equal(sm_rb_push(&one, &v), true);
    zassert_equal(sm_rb_count(&one), 1);
    zassert_equal(sm_rb_read(&one, out, 1), 1);
    zassert_equal(out[0], 22);
    sm_rb_deinit(&one);
}

ZTEST(ringbuf, non_power_of_two_capacity_is_supported)
{
    /* The product capacities are 100 and 20. Neither is a power of two, so
     * the implementation must not depend on mask-based wrapping. */
    sm_rb_t odd;
    uint8_t s[100];
    uint8_t out[100];
    int     i;

    zassert_equal(sm_rb_init(&odd, s, 1, 100), 0);
    for (i = 0; i < 250; i++) {
        uint8_t v = (uint8_t)i;
        sm_rb_push(&odd, &v);
    }
    zassert_equal(sm_rb_count(&odd), 100);
    zassert_equal(sm_rb_read(&odd, out, 100), 100);
    for (i = 0; i < 100; i++) {
        zassert_equal(out[i], (uint8_t)(150 + i)); /* latest 100 of 0..249 */
    }
    sm_rb_deinit(&odd);
}

ZTEST_SUITE(ringbuf, NULL, NULL, NULL, NULL, NULL);
