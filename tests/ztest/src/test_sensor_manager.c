/*
 * Sensor manager tests: rate control, sequencing, retention, gap reporting.
 * Ported from tests/test_sensor_manager.c to Ztest. Injected virtual time keeps
 * every case deterministic; the SM_PORT_ZEPHYR locks are exercised but never
 * contended here. Assertion intent is unchanged from the original.
 */
#include <string.h>

#include <zephyr/ztest.h>

#include "sensor_manager.h"
#include "sensor_sim.h"

static sensor_manager_t mgr;

static const int16_t A[3] = {1, 2, 3};
static const int16_t G[3] = {4, 5, 6};

/* Virtual-time run length shared by the drift-free and simulated-run cases. */
#define TEN_SECONDS_US 10000000ull
#define POLL_STEP_US   1000ull

/* ---------------- rate control ---------------- */

ZTEST(sensor_manager, rate_not_due_before_first_period)
{
    sm_rate_t r;

    sm_rate_init(&r, 50, 0);
    zassert_equal(sm_rate_poll(&r, 0), 0);
    zassert_equal(sm_rate_poll(&r, 19999), 0);
    zassert_equal(sm_rate_poll(&r, 20000), 1); /* 50 Hz => 20 ms */
}

/*
 * The headline property: exactly rate*seconds firings, with no accumulated
 * drift, even when every poll arrives slightly late.
 */
ZTEST(sensor_manager, rate_is_drift_free_over_ten_seconds)
{
    sm_rate_t imu, temp;
    uint32_t  imu_total = 0, temp_total = 0;
    uint64_t  t;

    sm_rate_init(&imu, SM_IMU_RATE_HZ, 0);
    sm_rate_init(&temp, SM_TEMP_RATE_HZ, 0);

    /* Poll every 1 ms with a 300 us jitter that always runs late. A naive
     * "next = now + period" scheduler would lose ~15 samples over this run. */
    for (t = 0; t <= TEN_SECONDS_US; t += POLL_STEP_US) {
        imu_total += sm_rate_poll(&imu, t + 300);
        temp_total += sm_rate_poll(&temp, t + 300);
    }

    zassert_equal(imu_total, 500); /* 50 Hz x 10 s, exactly */
    zassert_equal(temp_total, 10); /*  1 Hz x 10 s, exactly */
    zassert_equal(imu.missed, 0);
    zassert_equal(temp.missed, 0);
}

ZTEST(sensor_manager, rate_catches_up_a_short_stall)
{
    sm_rate_t r;

    sm_rate_init(&r, 50, 0);
    /* 60 ms late => 3 periods due, within the catch-up clamp. */
    zassert_equal(sm_rate_poll(&r, 60000), 3);
    zassert_equal(r.missed, 0);
}

/*
 * A long stall must not produce a burst that evicts the recent data still in
 * the buffer. The clamp trades completeness for recency and reports the loss.
 */
ZTEST(sensor_manager, rate_clamps_a_long_stall_and_reports_it)
{
    sm_rate_t r;
    uint32_t  due;

    sm_rate_init(&r, 50, 0);
    due = sm_rate_poll(&r, 2000000); /* 2 s stall = 100 periods */

    zassert_equal(due, SM_RATE_MAX_CATCHUP);
    zassert_equal(r.missed, 100 - SM_RATE_MAX_CATCHUP);

    /* Phase must have resynchronised to the stall's end, not stayed behind. */
    zassert_equal(sm_rate_poll(&r, 2000000), 0);
    zassert_equal(sm_rate_poll(&r, 2020000), 1);
}

ZTEST(sensor_manager, rate_zero_hz_never_fires)
{
    sm_rate_t r;

    sm_rate_init(&r, 0, 0);
    zassert_equal(sm_rate_poll(&r, 1000000000ull), 0);
}

/* ---------------- manager ---------------- */

ZTEST(sensor_manager, init_gives_empty_buffers_at_configured_capacity)
{
    sm_stats_t st;

    zassert_equal(sm_init(&mgr, 0), SM_OK);
    sm_get_stats(&mgr, &st);

    zassert_equal(st.imu.capacity, SM_IMU_CAPACITY);
    zassert_equal(st.temp.capacity, SM_TEMP_CAPACITY);
    zassert_equal(st.imu.count, 0);
    zassert_equal(st.temp.count, 0);
    zassert_equal(sm_init(NULL, 0), SM_ERR_ARG);

    sm_deinit(&mgr);
}

ZTEST(sensor_manager, submitted_samples_round_trip_intact)
{
    sm_imu_sample_t  out[4];
    sm_temp_sample_t tout[4];

    zassert_equal(sm_init(&mgr, 0), SM_OK);
    sm_submit_imu(&mgr, A, G, 1234);
    sm_submit_temp(&mgr, 33500, 5678);

    zassert_equal(sm_read_imu(&mgr, out, 4), 1);
    zassert_equal(out[0].t_us, 1234);
    zassert_equal(out[0].accel[0], 1);
    zassert_equal(out[0].accel[2], 3);
    zassert_equal(out[0].gyro[0], 4);
    zassert_equal(out[0].gyro[2], 6);

    zassert_equal(sm_read_temp(&mgr, tout, 4), 1);
    zassert_equal(tout[0].t_us, 5678);
    zassert_equal(tout[0].milli_celsius, 33500);

    sm_deinit(&mgr);
}

ZTEST(sensor_manager, sequence_numbers_are_monotonic_and_dense_when_not_dropping)
{
    sm_imu_sample_t out[10];
    int             i;

    zassert_equal(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 10; i++) {
        sm_submit_imu(&mgr, A, G, (uint64_t)i * 20000);
    }
    zassert_equal(sm_read_imu(&mgr, out, 10), 10);
    for (i = 0; i < 10; i++) {
        zassert_equal(out[i].seq, i);
    }
    sm_deinit(&mgr);
}

ZTEST(sensor_manager, imu_retains_latest_100)
{
    sm_imu_sample_t out[SM_IMU_CAPACITY];
    sm_stats_t      st;
    int             i;

    zassert_equal(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 150; i++) {
        sm_submit_imu(&mgr, A, G, (uint64_t)i * 20000);
    }

    sm_get_stats(&mgr, &st);
    zassert_equal(st.imu.count, SM_IMU_CAPACITY);
    zassert_equal(st.imu.pushed, 150);
    zassert_equal(st.imu.dropped, 50);

    zassert_equal(sm_read_imu(&mgr, out, SM_IMU_CAPACITY), SM_IMU_CAPACITY);
    zassert_equal(out[0].seq, 50); /* the 50 oldest were evicted */
    zassert_equal(out[99].seq, 149);
    sm_deinit(&mgr);
}

ZTEST(sensor_manager, temp_retains_latest_20)
{
    sm_temp_sample_t out[SM_TEMP_CAPACITY];
    sm_stats_t       st;
    int              i;

    zassert_equal(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 35; i++) {
        sm_submit_temp(&mgr, 33000 + i, (uint64_t)i * 1000000);
    }

    sm_get_stats(&mgr, &st);
    zassert_equal(st.temp.count, SM_TEMP_CAPACITY);
    zassert_equal(st.temp.dropped, 15);

    zassert_equal(sm_read_temp(&mgr, out, SM_TEMP_CAPACITY), SM_TEMP_CAPACITY);
    zassert_equal(out[0].milli_celsius, 33000 + 15);
    zassert_equal(out[19].milli_celsius, 33000 + 34);
    sm_deinit(&mgr);
}

/*
 * Overflow must be *detectable downstream*, not just survivable. The gap in
 * sequence numbers across a drop is what the phone uses to know its timeline
 * is incomplete. This is the difference between graceful and silent.
 */
ZTEST(sensor_manager, overflow_leaves_a_detectable_sequence_gap)
{
    sm_imu_sample_t out[SM_IMU_CAPACITY];
    uint32_t        last_seq;
    int             i;

    zassert_equal(sm_init(&mgr, 0), SM_OK);

    for (i = 0; i < 10; i++) {
        sm_submit_imu(&mgr, A, G, (uint64_t)i * 20000);
    }
    zassert_equal(sm_read_imu(&mgr, out, 10), 10);
    last_seq = out[9].seq;

    /* Consumer goes away (phone out of range) and the buffer overruns. */
    for (i = 10; i < 10 + 250; i++) {
        sm_submit_imu(&mgr, A, G, (uint64_t)i * 20000);
    }

    zassert_equal(sm_read_imu(&mgr, out, SM_IMU_CAPACITY), SM_IMU_CAPACITY);
    zassert_true(out[0].seq > last_seq + 1);         /* a gap exists */
    zassert_equal(out[0].seq - last_seq - 1, 150);   /* and it is exactly 150 */
    sm_deinit(&mgr);
}

ZTEST(sensor_manager, streams_are_independent)
{
    sm_stats_t st;
    int        i;

    zassert_equal(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 200; i++) {
        sm_submit_imu(&mgr, A, G, (uint64_t)i);
    }
    sm_submit_temp(&mgr, 33000, 1);

    sm_get_stats(&mgr, &st);
    zassert_equal(st.imu.dropped, 100);
    zassert_equal(st.temp.dropped, 0); /* IMU overrun must not disturb temp */
    zassert_equal(st.temp.count, 1);
    sm_deinit(&mgr);
}

ZTEST(sensor_manager, peek_release_supports_a_retried_notification)
{
    sm_imu_sample_t out[8];
    int             i;

    zassert_equal(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 8; i++) {
        sm_submit_imu(&mgr, A, G, (uint64_t)i);
    }

    /* First attempt: stack refuses, nothing released. */
    zassert_equal(sm_peek_imu(&mgr, out, 8), 8);
    zassert_equal(sm_pending_imu(&mgr), 8);

    /* Retry: stack accepts 5 of them. */
    zassert_equal(sm_peek_imu(&mgr, out, 8), 8);
    zassert_equal(out[0].seq, 0);
    zassert_equal(sm_release_imu(&mgr, 5), 5);
    zassert_equal(sm_pending_imu(&mgr), 3);
    zassert_equal(sm_peek_imu(&mgr, out, 8), 3);
    zassert_equal(out[0].seq, 5); /* resumes exactly where it left off */

    sm_deinit(&mgr);
}

/* The temperature stream has its own peek/release entry points; exercise them
 * directly rather than assuming they mirror the IMU path they wrap. */
ZTEST(sensor_manager, temp_peek_release_supports_a_retried_notification)
{
    sm_temp_sample_t out[8];
    int              i;

    zassert_equal(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 6; i++) {
        sm_submit_temp(&mgr, 33000 + i, (uint64_t)i * 1000000ull);
    }

    /* First attempt is peeked but not released: everything stays pending. */
    zassert_equal(sm_peek_temp(&mgr, out, 8), 6);
    zassert_equal(sm_pending_temp(&mgr), 6);
    zassert_equal(out[0].milli_celsius, 33000);

    /* Stack accepts 4; the remainder must resume exactly where it left off. */
    zassert_equal(sm_release_temp(&mgr, 4), 4);
    zassert_equal(sm_pending_temp(&mgr), 2);
    zassert_equal(sm_peek_temp(&mgr, out, 8), 2);
    zassert_equal(out[0].milli_celsius, 33000 + 4);

    sm_deinit(&mgr);
}

/* End-to-end: drive the manager from the simulator through 10 s of virtual
 * time and assert the sample counts the requirements specify. */
ZTEST(sensor_manager, simulated_ten_second_run_produces_expected_rates)
{
    sensor_sim_t sim;
    sm_stats_t   st;
    uint64_t     t;

    zassert_equal(sm_init(&mgr, 0), SM_OK);
    sensor_sim_init(&sim, 0xC0FFEEu);

    for (t = 0; t <= TEN_SECONDS_US; t += POLL_STEP_US) {
        uint32_t n = sm_rate_poll(&mgr.imu_rate, t);
        while (n--) {
            int16_t a[3], g[3];
            sensor_sim_imu(&sim, a, g);
            sm_submit_imu(&mgr, a, g, t);
        }
        n = sm_rate_poll(&mgr.temp_rate, t);
        while (n--) {
            sm_submit_temp(&mgr, sensor_sim_temp(&sim), t);
        }
    }

    sm_get_stats(&mgr, &st);
    zassert_equal(st.imu.pushed, 500);
    zassert_equal(st.temp.pushed, 10);
    /* Nothing was ever drained, so the buffers are full and the excess is
     * accounted for exactly. */
    zassert_equal(st.imu.count, SM_IMU_CAPACITY);
    zassert_equal(st.imu.dropped, 500 - SM_IMU_CAPACITY);
    zassert_equal(st.temp.count, 10);
    zassert_equal(st.temp.dropped, 0);
    sm_deinit(&mgr);
}

ZTEST(sensor_manager, simulator_is_deterministic)
{
    sensor_sim_t a, b;
    int          i;

    sensor_sim_init(&a, 42);
    sensor_sim_init(&b, 42);
    for (i = 0; i < 100; i++) {
        int16_t aa[3], ag[3], ba[3], bg[3];
        sensor_sim_imu(&a, aa, ag);
        sensor_sim_imu(&b, ba, bg);
        zassert_equal(memcmp(aa, ba, sizeof(aa)), 0);
        zassert_equal(memcmp(ag, bg, sizeof(ag)), 0);
    }
}

ZTEST(sensor_manager, simulated_signals_are_in_plausible_range)
{
    sensor_sim_t sim;
    int          i;

    sensor_sim_init(&sim, 7);
    for (i = 0; i < 500; i++) {
        int16_t a[3], g[3];
        int32_t c;

        sensor_sim_imu(&sim, a, g);
        c = sensor_sim_temp(&sim);

        /* Z axis should sit near 1 g (4096 LSB at +/-8g FS). */
        zassert_true(a[2] > 3800 && a[2] < 4400);
        /* Skin temperature stays in a physiological band. */
        zassert_true(c > 28000 && c < 38000);
    }
}

ZTEST_SUITE(sensor_manager, NULL, NULL, NULL, NULL, NULL);
