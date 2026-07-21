/*
 * BLE batch encoder tests. These pin the wire format, which is a contract
 * with a mobile team building in parallel: if a field moves, a test fails
 * here rather than a bug appearing in someone else's app three weeks later.
 */
#include "test_harness.h"

#include "ble_batch.h"
#include "sensor_manager.h"

static sensor_manager_t mgr;

static void fill_imu(sensor_manager_t *m, int n, uint64_t period_us)
{
    int i;

    for (i = 0; i < n; i++) {
        int16_t a[3] = {(int16_t)(100 + i), (int16_t)(-200 - i), 4096};
        int16_t g[3] = {(int16_t)(i), (int16_t)(-i), 0};
        sm_submit_imu(m, a, g, (uint64_t)i * period_us);
    }
}

TEST(imu_frame_header_and_records_are_exact)
{
    sm_imu_sample_t samples[32];
    uint8_t         frame[BLE_DEFAULT_PAYLOAD];
    size_t          consumed, len, n;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    fill_imu(&mgr, 4, 20000);

    n = sm_read_imu(&mgr, samples, 32);
    CHECK_EQ(n, 4);

    len = ble_batch_encode_imu(samples, n, 0, frame, sizeof(frame), &consumed);
    CHECK_EQ(consumed, 4);
    CHECK_EQ(len, BLE_BATCH_HDR_LEN + 4 * BLE_IMU_REC_LEN);

    CHECK_EQ(ble_batch_type(frame), BLE_BATCH_TYPE_IMU);
    CHECK_EQ(ble_batch_count(frame), 4);
    CHECK_EQ(ble_batch_first_seq(frame), 0);
    CHECK_EQ(ble_batch_base_time(frame), 0);
    CHECK_EQ(ble_batch_dropped(frame), 0);

    /* Record 0: dt 0 by definition, accel.x 100 (little-endian). */
    CHECK_EQ(frame[16] | (frame[17] << 8), 0);
    CHECK_EQ((int16_t)(frame[18] | (frame[19] << 8)), 100);
    /* Record 2: dt is the gap from record 1, i.e. one 50 Hz period. */
    {
        size_t off = BLE_BATCH_HDR_LEN + 2 * BLE_IMU_REC_LEN;
        CHECK_EQ(frame[off] | (frame[off + 1] << 8), 20000);
    }

    sm_deinit(&mgr);
}

/* Negative accelerations must survive the round trip as signed values. */
TEST(negative_values_encode_as_twos_complement)
{
    sm_imu_sample_t samples[4];
    uint8_t         frame[BLE_DEFAULT_PAYLOAD];
    size_t          consumed;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    fill_imu(&mgr, 1, 20000);
    sm_read_imu(&mgr, samples, 4);
    ble_batch_encode_imu(samples, 1, 0, frame, sizeof(frame), &consumed);

    /* accel.y was -200 */
    CHECK_EQ((int16_t)(frame[20] | (frame[21] << 8)), -200);
    sm_deinit(&mgr);
}

/*
 * The batching claim from the architecture doc, asserted rather than
 * asserted-in-prose: a 244-byte payload carries 16 IMU samples, which turns
 * 50 notifications/s into ~3.1/s at 50 Hz.
 */
TEST(payload_budget_yields_sixteen_imu_samples)
{
    sm_imu_sample_t samples[64];
    uint8_t         frame[BLE_DEFAULT_PAYLOAD];
    size_t          consumed, len, n;

    CHECK_EQ(BLE_IMU_MAX_RECORDS(BLE_DEFAULT_PAYLOAD), 16);

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    fill_imu(&mgr, 40, 20000);
    n = sm_read_imu(&mgr, samples, 64);
    CHECK_EQ(n, 40);

    len = ble_batch_encode_imu(samples, n, 0, frame, sizeof(frame), &consumed);
    CHECK_EQ(consumed, 16); /* capped by payload, not by n */
    CHECK(len <= BLE_DEFAULT_PAYLOAD);
    CHECK_EQ(ble_batch_count(frame), 16);
    sm_deinit(&mgr);
}

/* Partial packing must not lose the remainder: the caller releases only what
 * was consumed, so the rest goes out next frame. */
TEST(unconsumed_samples_survive_for_the_next_frame)
{
    sm_imu_sample_t samples[64];
    uint8_t         frame[BLE_DEFAULT_PAYLOAD];
    size_t          consumed;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    fill_imu(&mgr, 40, 20000);

    CHECK_EQ(sm_peek_imu(&mgr, samples, 64), 40);
    ble_batch_encode_imu(samples, 40, 0, frame, sizeof(frame), &consumed);
    CHECK_EQ(sm_release_imu(&mgr, consumed), 16);
    CHECK_EQ(sm_pending_imu(&mgr), 24);

    CHECK_EQ(sm_peek_imu(&mgr, samples, 64), 24);
    ble_batch_encode_imu(samples, 24, 0, frame, sizeof(frame), &consumed);
    CHECK_EQ(ble_batch_first_seq(frame), 16); /* continues the sequence */
    sm_deinit(&mgr);
}

/*
 * A gap longer than the u16 microsecond window must close the batch rather
 * than emit a wrapped delta. This is the "sensor was suspended" case.
 */
TEST(oversized_time_delta_closes_the_batch_early)
{
    sm_imu_sample_t samples[8];
    uint8_t         frame[BLE_DEFAULT_PAYLOAD];
    size_t          consumed, n;
    int16_t         a[3] = {1, 2, 3}, g[3] = {4, 5, 6};

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    sm_submit_imu(&mgr, a, g, 0);
    sm_submit_imu(&mgr, a, g, 20000);
    sm_submit_imu(&mgr, a, g, 5000000); /* 5 s later: exceeds u16 µs */

    n = sm_read_imu(&mgr, samples, 8);
    CHECK_EQ(n, 3);
    ble_batch_encode_imu(samples, n, 0, frame, sizeof(frame), &consumed);
    CHECK_EQ(consumed, 2); /* third sample deferred */
    CHECK_EQ(ble_batch_count(frame), 2);
    sm_deinit(&mgr);
}

TEST(temp_frame_uses_millisecond_deltas)
{
    sm_temp_sample_t samples[24];
    uint8_t          frame[BLE_DEFAULT_PAYLOAD];
    size_t           consumed, len, n;
    int              i;

    /* A full 244 B payload holds 38 temp records; 20 comfortably fit one. */
    CHECK_EQ(BLE_TEMP_MAX_RECORDS(BLE_DEFAULT_PAYLOAD), 38);

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    for (i = 0; i < 20; i++) {
        sm_submit_temp(&mgr, 33000 + i * 10, (uint64_t)i * 1000000ull);
    }
    n = sm_read_temp(&mgr, samples, 24);
    CHECK_EQ(n, 20);

    len = ble_batch_encode_temp(samples, n, 0, frame, sizeof(frame), &consumed);
    /* All 20 fit: 16 + 20*6 = 136 bytes, and 19 s of span fits u16 ms. */
    CHECK_EQ(consumed, 20);
    CHECK_EQ(len, 136);
    CHECK_EQ(ble_batch_type(frame), BLE_BATCH_TYPE_TEMP);

    /* Record 3 is one 1 Hz period after record 2 => dt_ms 1000. */
    {
        size_t off = BLE_BATCH_HDR_LEN + 3 * BLE_TEMP_REC_LEN;
        CHECK_EQ(frame[off] | (frame[off + 1] << 8), 1000);
    }
    sm_deinit(&mgr);
}

/*
 * The temperature stream encodes deltas in milliseconds, so its overflow
 * threshold is a gap wider than 65.535 s, not 65.5 ms. A gap past the u16 ms
 * window must close the batch early exactly as the IMU path does for µs.
 */
TEST(temp_oversized_time_delta_closes_the_batch_early)
{
    sm_temp_sample_t samples[8];
    uint8_t          frame[BLE_DEFAULT_PAYLOAD];
    size_t           consumed, n;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    sm_submit_temp(&mgr, 33000, 0);
    sm_submit_temp(&mgr, 33100, 1000000ull);   /* +1 s: fits u16 ms */
    sm_submit_temp(&mgr, 33200, 200000000ull); /* +199 s: exceeds u16 ms */

    n = sm_read_temp(&mgr, samples, 8);
    CHECK_EQ(n, 3);
    CHECK_EQ(ble_batch_encode_temp(samples, n, 0, frame, sizeof(frame), &consumed),
             (size_t)(BLE_BATCH_HDR_LEN + 2 * BLE_TEMP_REC_LEN));
    CHECK_EQ(consumed, 2); /* third sample deferred */
    CHECK_EQ(ble_batch_count(frame), 2);
    sm_deinit(&mgr);
}

/* The temp encoder has its own argument-validation copy; pin it independently
 * of the IMU one so a change to either is caught. */
TEST(temp_encoder_rejects_degenerate_inputs)
{
    sm_temp_sample_t samples[4];
    uint8_t          frame[BLE_DEFAULT_PAYLOAD];
    uint8_t          tiny[BLE_BATCH_HDR_LEN + BLE_TEMP_REC_LEN - 1];
    size_t           consumed;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    sm_submit_temp(&mgr, 33000, 0);
    sm_submit_temp(&mgr, 33001, 1000000ull);
    sm_read_temp(&mgr, samples, 4);

    CHECK_EQ(ble_batch_encode_temp(samples, 0, 0, frame, sizeof(frame), &consumed), 0);
    CHECK_EQ(consumed, 0);
    CHECK_EQ(ble_batch_encode_temp(NULL, 2, 0, frame, sizeof(frame), &consumed), 0);
    CHECK_EQ(ble_batch_encode_temp(samples, 2, 0, NULL, sizeof(frame), &consumed), 0);
    /* Buffer too small for even a header plus one record. */
    CHECK_EQ(ble_batch_encode_temp(samples, 2, 0, tiny, sizeof(tiny), &consumed), 0);
    sm_deinit(&mgr);
}

TEST(dropped_count_reaches_the_phone_and_saturates)
{
    sm_imu_sample_t samples[SM_IMU_CAPACITY];
    uint8_t         frame[BLE_DEFAULT_PAYLOAD];
    sm_stats_t      st;
    size_t          consumed, n;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    fill_imu(&mgr, 130, 20000); /* 30 evicted */
    sm_get_stats(&mgr, &st);
    CHECK_EQ(st.imu.dropped, 30);

    n = sm_read_imu(&mgr, samples, SM_IMU_CAPACITY);
    ble_batch_encode_imu(samples, n, st.imu.dropped, frame, sizeof(frame), &consumed);
    CHECK_EQ(ble_batch_dropped(frame), 30);

    /* "At least 65535 lost" is true; a wrapped small number would be a lie. */
    ble_batch_encode_imu(samples, n, 1000000, frame, sizeof(frame), &consumed);
    CHECK_EQ(ble_batch_dropped(frame), 0xFFFF);
    sm_deinit(&mgr);
}

TEST(encoder_rejects_degenerate_inputs)
{
    sm_imu_sample_t samples[4];
    uint8_t         frame[BLE_DEFAULT_PAYLOAD];
    uint8_t         tiny[8];
    size_t          consumed;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    fill_imu(&mgr, 2, 20000);
    sm_read_imu(&mgr, samples, 4);

    CHECK_EQ(ble_batch_encode_imu(samples, 0, 0, frame, sizeof(frame), &consumed), 0);
    CHECK_EQ(consumed, 0);
    CHECK_EQ(ble_batch_encode_imu(NULL, 2, 0, frame, sizeof(frame), &consumed), 0);
    CHECK_EQ(ble_batch_encode_imu(samples, 2, 0, NULL, sizeof(frame), &consumed), 0);
    /* Buffer too small even for a header plus one record. */
    CHECK_EQ(ble_batch_encode_imu(samples, 2, 0, tiny, sizeof(tiny), &consumed), 0);
    sm_deinit(&mgr);
}

TEST_MAIN_BEGIN("ble_batch")
RUN(imu_frame_header_and_records_are_exact);
RUN(negative_values_encode_as_twos_complement);
RUN(payload_budget_yields_sixteen_imu_samples);
RUN(unconsumed_samples_survive_for_the_next_frame);
RUN(oversized_time_delta_closes_the_batch_early);
RUN(temp_frame_uses_millisecond_deltas);
RUN(temp_oversized_time_delta_closes_the_batch_early);
RUN(temp_encoder_rejects_degenerate_inputs);
RUN(dropped_count_reaches_the_phone_and_saturates);
RUN(encoder_rejects_degenerate_inputs);
TEST_MAIN_END()
