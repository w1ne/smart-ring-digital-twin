/*
 * Sensor manager tests: rate control, sequencing, retention, gap reporting.
 * SM_PORT_NONE + injected virtual time => fully deterministic.
 */
#include "test_harness.h"

#include "sensor_manager.h"
#include "sensor_sim.h"

static sensor_manager_t mgr;

static const int16_t A[3] = {1, 2, 3};
static const int16_t G[3] = {4, 5, 6};

/* ---------------- rate control ---------------- */

TEST(rate_not_due_before_first_period)
{
    sm_rate_t r;

    sm_rate_init(&r, 50, 0);
    CHECK_EQ(sm_rate_poll(&r, 0), 0);
    CHECK_EQ(sm_rate_poll(&r, 19999), 0);
    CHECK_EQ(sm_rate_poll(&r, 20000), 1); /* 50 Hz => 20 ms */
}

/*
 * The headline property: exactly rate*seconds firings, with no accumulated
 * drift, even when every poll arrives slightly late.
 */
TEST(rate_is_drift_free_over_ten_seconds)
{
    sm_rate_t imu, temp;
    uint32_t  imu_total = 0, temp_total = 0;
    uint64_t  t;

    sm_rate_init(&imu, SM_IMU_RATE_HZ, 0);
    sm_rate_init(&temp, SM_TEMP_RATE_HZ, 0);

    /* Poll every 1 ms with a 300 µs jitter that always runs late. A naive
     * "next = now + period" scheduler would lose ~15 samples over this run. */
    for (t = 0; t <= 10000000ull; t += 1000) {
        imu_total += sm_rate_poll(&imu, t + 300);
        temp_total += sm_rate_poll(&temp, t + 300);
    }

    CHECK_EQ(imu_total, 500); /* 50 Hz x 10 s, exactly */
    CHECK_EQ(temp_total, 10); /*  1 Hz x 10 s, exactly */
    CHECK_EQ(imu.missed, 0);
    CHECK_EQ(temp.missed, 0);
}

TEST(rate_catches_up_a_short_stall)
{
    sm_rate_t r;

    sm_rate_init(&r, 50, 0);
    /* 60 ms late => 3 periods due, within the catch-up clamp. */
    CHECK_EQ(sm_rate_poll(&r, 60000), 3);
    CHECK_EQ(r.missed, 0);
}

/*
 * A long stall must not produce a burst that evicts the recent data still in
 * the buffer. The clamp trades completeness for recency and reports the loss.
 */
TEST(rate_clamps_a_long_stall_and_reports_it)
{
    sm_rate_t r;
    uint32_t  due;

    sm_rate_init(&r, 50, 0);
    due = sm_rate_poll(&r, 2000000); /* 2 s stall = 100 periods */

    CHECK_EQ(due, SM_RATE_MAX_CATCHUP);
    CHECK_EQ(r.missed, 100 - SM_RATE_MAX_CATCHUP);

    /* Phase must have resynchronised to the stall's end, not stayed behind. */
    CHECK_EQ(sm_rate_poll(&r, 2000000), 0);
    CHECK_EQ(sm_rate_poll(&r, 2020000), 1);
}

TEST(rate_zero_hz_never_fires)
{
    sm_rate_t r;

    sm_rate_init(&r, 0, 0);
    CHECK_EQ(sm_rate_poll(&r, 1000000000ull), 0);
}

/* ---------------- manager ---------------- */

TEST(init_gives_empty_buffers_at_configured_capacity)
{
    sm_stats_t st;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    sm_get_stats(&mgr, &st);

    CHECK_EQ(st.imu.capacity, SM_IMU_CAPACITY);
    CHECK_EQ(st.temp.capacity, SM_TEMP_CAPACITY);
    CHECK_EQ(st.imu.count, 0);
    CHECK_EQ(st.temp.count, 0);
    CHECK_EQ(sm_init(NULL, 0), SM_ERR_ARG);

    sm_deinit(&mgr);
}

TEST(submitted_samples_round_trip_intact)
{
    sm_imu_sample_t  out[4];
    sm_temp_sample_t tout[4];

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    sm_submit_imu(&mgr, A, G, 1234);
    sm_submit_temp(&mgr, 33500, 5678);

    CHECK_EQ(sm_read_imu(&mgr, out, 4), 1);
    CHECK_EQ(out[0].t_us, 1234);
    CHECK_EQ(out[0].accel[0], 1);
    CHECK_EQ(out[0].accel[2], 3);
    CHECK_EQ(out[0].gyro[0], 4);
    CHECK_EQ(out[0].gyro[2], 6);

    CHECK_EQ(sm_read_temp(&mgr, tout, 4), 1);
    CHECK_EQ(tout[0].t_us, 5678);
    CHECK_EQ(tout[0].milli_celsius, 33500);

    sm_deinit(&mgr);
}

TEST(sequence_numbers_are_monotonic_and_dense_when_not_dropping)
{
    sm_imu_sample_t out[10];
    int             i;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 10; i++) {
        sm_submit_imu(&mgr, A, G, (uint64_t)i * 20000);
    }
    CHECK_EQ(sm_read_imu(&mgr, out, 10), 10);
    for (i = 0; i < 10; i++) {
        CHECK_EQ(out[i].seq, i);
    }
    sm_deinit(&mgr);
}

TEST(imu_retains_latest_100)
{
    sm_imu_sample_t out[SM_IMU_CAPACITY];
    sm_stats_t      st;
    int             i;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 150; i++) {
        sm_submit_imu(&mgr, A, G, (uint64_t)i * 20000);
    }

    sm_get_stats(&mgr, &st);
    CHECK_EQ(st.imu.count, SM_IMU_CAPACITY);
    CHECK_EQ(st.imu.pushed, 150);
    CHECK_EQ(st.imu.dropped, 50);

    CHECK_EQ(sm_read_imu(&mgr, out, SM_IMU_CAPACITY), SM_IMU_CAPACITY);
    CHECK_EQ(out[0].seq, 50); /* the 50 oldest were evicted */
    CHECK_EQ(out[99].seq, 149);
    sm_deinit(&mgr);
}

TEST(temp_retains_latest_20)
{
    sm_temp_sample_t out[SM_TEMP_CAPACITY];
    sm_stats_t       st;
    int              i;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 35; i++) {
        sm_submit_temp(&mgr, 33000 + i, (uint64_t)i * 1000000);
    }

    sm_get_stats(&mgr, &st);
    CHECK_EQ(st.temp.count, SM_TEMP_CAPACITY);
    CHECK_EQ(st.temp.dropped, 15);

    CHECK_EQ(sm_read_temp(&mgr, out, SM_TEMP_CAPACITY), SM_TEMP_CAPACITY);
    CHECK_EQ(out[0].milli_celsius, 33000 + 15);
    CHECK_EQ(out[19].milli_celsius, 33000 + 34);
    sm_deinit(&mgr);
}

/*
 * Overflow must be *detectable downstream*, not just survivable. The gap in
 * sequence numbers across a drop is what the phone uses to know its timeline
 * is incomplete. This is the difference between graceful and silent.
 */
TEST(overflow_leaves_a_detectable_sequence_gap)
{
    sm_imu_sample_t out[SM_IMU_CAPACITY];
    uint32_t        last_seq;
    int             i;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);

    for (i = 0; i < 10; i++) {
        sm_submit_imu(&mgr, A, G, (uint64_t)i * 20000);
    }
    CHECK_EQ(sm_read_imu(&mgr, out, 10), 10);
    last_seq = out[9].seq;

    /* Consumer goes away (phone out of range) and the buffer overruns. */
    for (i = 10; i < 10 + 250; i++) {
        sm_submit_imu(&mgr, A, G, (uint64_t)i * 20000);
    }

    CHECK_EQ(sm_read_imu(&mgr, out, SM_IMU_CAPACITY), SM_IMU_CAPACITY);
    CHECK(out[0].seq > last_seq + 1);         /* a gap exists */
    CHECK_EQ(out[0].seq - last_seq - 1, 150); /* and it is exactly 150 */
    sm_deinit(&mgr);
}

TEST(streams_are_independent)
{
    sm_stats_t st;
    int        i;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 200; i++) {
        sm_submit_imu(&mgr, A, G, (uint64_t)i);
    }
    sm_submit_temp(&mgr, 33000, 1);

    sm_get_stats(&mgr, &st);
    CHECK_EQ(st.imu.dropped, 100);
    CHECK_EQ(st.temp.dropped, 0); /* IMU overrun must not disturb temp */
    CHECK_EQ(st.temp.count, 1);
    sm_deinit(&mgr);
}

TEST(peek_release_supports_a_retried_notification)
{
    sm_imu_sample_t out[8];
    int             i;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 8; i++) {
        sm_submit_imu(&mgr, A, G, (uint64_t)i);
    }

    /* First attempt: stack refuses, nothing released. */
    CHECK_EQ(sm_peek_imu(&mgr, out, 8), 8);
    CHECK_EQ(sm_pending_imu(&mgr), 8);

    /* Retry: stack accepts 5 of them. */
    CHECK_EQ(sm_peek_imu(&mgr, out, 8), 8);
    CHECK_EQ(out[0].seq, 0);
    CHECK_EQ(sm_release_imu(&mgr, 5), 5);
    CHECK_EQ(sm_pending_imu(&mgr), 3);
    CHECK_EQ(sm_peek_imu(&mgr, out, 8), 3);
    CHECK_EQ(out[0].seq, 5); /* resumes exactly where it left off */

    sm_deinit(&mgr);
}

/* The temperature stream has its own peek/release entry points; exercise them
 * directly rather than assuming they mirror the IMU path they wrap. */
TEST(temp_peek_release_supports_a_retried_notification)
{
    sm_temp_sample_t out[8];
    int              i;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 6; i++) {
        sm_submit_temp(&mgr, 33000 + i, (uint64_t)i * 1000000ull);
    }

    /* First attempt is peeked but not released: everything stays pending. */
    CHECK_EQ(sm_peek_temp(&mgr, out, 8), 6);
    CHECK_EQ(sm_pending_temp(&mgr), 6);
    CHECK_EQ(out[0].milli_celsius, 33000);

    /* Stack accepts 4; the remainder must resume exactly where it left off. */
    CHECK_EQ(sm_release_temp(&mgr, 4), 4);
    CHECK_EQ(sm_pending_temp(&mgr), 2);
    CHECK_EQ(sm_peek_temp(&mgr, out, 8), 2);
    CHECK_EQ(out[0].milli_celsius, 33000 + 4);

    sm_deinit(&mgr);
}

/* End-to-end: drive the manager from the simulator through 10 s of virtual
 * time and assert the sample counts the requirements specify. */
TEST(simulated_ten_second_run_produces_expected_rates)
{
    sensor_sim_t sim;
    sm_stats_t   st;
    uint64_t     t;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    sensor_sim_init(&sim, 0xC0FFEEu);

    for (t = 0; t <= 10000000ull; t += 1000) {
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
    CHECK_EQ(st.imu.pushed, 500);
    CHECK_EQ(st.temp.pushed, 10);
    /* Nothing was ever drained, so the buffers are full and the excess is
     * accounted for exactly. */
    CHECK_EQ(st.imu.count, SM_IMU_CAPACITY);
    CHECK_EQ(st.imu.dropped, 500 - SM_IMU_CAPACITY);
    CHECK_EQ(st.temp.count, 10);
    CHECK_EQ(st.temp.dropped, 0);
    sm_deinit(&mgr);
}

TEST(simulator_is_deterministic)
{
    sensor_sim_t a, b;
    int          i;

    sensor_sim_init(&a, 42);
    sensor_sim_init(&b, 42);
    for (i = 0; i < 100; i++) {
        int16_t aa[3], ag[3], ba[3], bg[3];
        sensor_sim_imu(&a, aa, ag);
        sensor_sim_imu(&b, ba, bg);
        CHECK_EQ(memcmp(aa, ba, sizeof(aa)), 0);
        CHECK_EQ(memcmp(ag, bg, sizeof(ag)), 0);
    }
}

TEST(simulated_signals_are_in_plausible_range)
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
        CHECK(a[2] > 3800 && a[2] < 4400);
        /* Skin temperature stays in a physiological band. */
        CHECK(c > 28000 && c < 38000);
    }
}

TEST_MAIN_BEGIN("sensor_manager")
RUN(rate_not_due_before_first_period);
RUN(rate_is_drift_free_over_ten_seconds);
RUN(rate_catches_up_a_short_stall);
RUN(rate_clamps_a_long_stall_and_reports_it);
RUN(rate_zero_hz_never_fires);
RUN(init_gives_empty_buffers_at_configured_capacity);
RUN(submitted_samples_round_trip_intact);
RUN(sequence_numbers_are_monotonic_and_dense_when_not_dropping);
RUN(imu_retains_latest_100);
RUN(temp_retains_latest_20);
RUN(overflow_leaves_a_detectable_sequence_gap);
RUN(streams_are_independent);
RUN(peek_release_supports_a_retried_notification);
RUN(temp_peek_release_supports_a_retried_notification);
RUN(simulated_ten_second_run_produces_expected_rates);
RUN(simulator_is_deterministic);
RUN(simulated_signals_are_in_plausible_range);
TEST_MAIN_END()
