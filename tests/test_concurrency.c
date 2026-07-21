/*
 * Concurrency stress. Built against SM_PORT_POSIX and intended to be run
 * both normally and under ThreadSanitizer (`ctest -R tsan`, or the
 * smart_ring_tests_tsan targets).
 *
 * Two properties are checked, and they are different things:
 *
 *   1. Memory safety / no data races  - TSan's job. A clean TSan run over a
 *      real workload is worth more than any assertion we could write.
 *   2. Accounting integrity          - our job. Every sample that entered the
 *      manager must be accounted for exactly once as delivered, dropped, or
 *      still buffered. A ring buffer that loses a sample without incrementing
 *      `dropped` is the exact bug this test exists to catch, and it is
 *      invisible to a sanitizer.
 *
 * Records also carry an internal consistency relation so a torn read (a
 * consumer observing half of one sample and half of another) is detectable.
 */
#include "test_harness.h"

#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <unistd.h>

#include "sensor_manager.h"

#define STRESS_SAMPLES 20000

static sensor_manager_t mgr;

/* Encode a value into a sample such that any partially-written record is
 * detectable from the record alone. */
static void encode(int16_t v, int16_t accel[3], int16_t gyro[3])
{
    accel[0] = v;
    accel[1] = (int16_t)-v;
    accel[2] = (int16_t)(v ^ 0x5A5A);
    gyro[0]  = (int16_t)(v + 1);
    gyro[1]  = (int16_t)(v + 2);
    gyro[2]  = (int16_t)(v + 3);
}

static bool record_is_coherent(const sm_imu_sample_t *s)
{
    int16_t v = s->accel[0];

    return s->accel[1] == (int16_t)-v && s->accel[2] == (int16_t)(v ^ 0x5A5A) &&
           s->gyro[0] == (int16_t)(v + 1) && s->gyro[1] == (int16_t)(v + 2) &&
           s->gyro[2] == (int16_t)(v + 3);
}

/* ---- shared state ---- */

static atomic_int  g_producers_done;
static atomic_uint g_delivered;
static atomic_uint g_incoherent;
static atomic_uint g_seq_regressions;
static atomic_uint g_temp_delivered;

static void *imu_producer(void *arg)
{
    int n = *(int *)arg;
    int i;

    for (i = 0; i < n; i++) {
        int16_t a[3], g[3];
        encode((int16_t)i, a, g);
        sm_submit_imu(&mgr, a, g, (uint64_t)i * 20000ull);
        if ((i & 0x3F) == 0) {
            sched_yield(); /* widen the interleaving window */
        }
    }
    atomic_fetch_add(&g_producers_done, 1);
    return NULL;
}

static void *temp_producer(void *arg)
{
    int n = *(int *)arg;
    int i;

    for (i = 0; i < n; i++) {
        sm_submit_temp(&mgr, 33000 + (i % 500), (uint64_t)i * 1000000ull);
        if ((i & 0x1F) == 0) {
            sched_yield();
        }
    }
    atomic_fetch_add(&g_producers_done, 1);
    return NULL;
}

/* Drains both streams the way the BLE thread would: in bounded batches,
 * repeatedly, until the producers stop and the buffers are empty. */
static void *consumer(void *arg)
{
    int      expected_producers = *(int *)arg;
    uint32_t last_seq           = 0;
    bool     have_last          = false;

    for (;;) {
        sm_imu_sample_t  ibuf[16];
        sm_temp_sample_t tbuf[16];
        size_t           n, i;
        bool             idle;

        n = sm_read_imu(&mgr, ibuf, 16);
        for (i = 0; i < n; i++) {
            if (!record_is_coherent(&ibuf[i])) {
                atomic_fetch_add(&g_incoherent, 1);
            }
            /* Sequence numbers must never go backwards within a stream. */
            if (have_last && ibuf[i].seq < last_seq) {
                atomic_fetch_add(&g_seq_regressions, 1);
            }
            last_seq  = ibuf[i].seq;
            have_last = true;
        }
        atomic_fetch_add(&g_delivered, (unsigned)n);
        idle = (n == 0);

        n = sm_read_temp(&mgr, tbuf, 16);
        atomic_fetch_add(&g_temp_delivered, (unsigned)n);
        idle = idle && (n == 0);

        if (idle && atomic_load(&g_producers_done) == expected_producers) {
            break; /* producers finished and buffers are drained */
        }
        if (idle) {
            sched_yield();
        }
    }
    return NULL;
}

static void reset_counters(void)
{
    atomic_store(&g_producers_done, 0);
    atomic_store(&g_delivered, 0);
    atomic_store(&g_incoherent, 0);
    atomic_store(&g_seq_regressions, 0);
    atomic_store(&g_temp_delivered, 0);
}

/*
 * One producer per stream plus a shared consumer: the production topology.
 */
TEST(concurrent_producers_and_consumer_account_for_every_sample)
{
    pthread_t  t_imu, t_temp, t_con;
    int        n         = STRESS_SAMPLES;
    int        temp_n    = STRESS_SAMPLES / 20;
    int        producers = 2;
    sm_stats_t st;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    reset_counters();

    pthread_create(&t_con, NULL, consumer, &producers);
    pthread_create(&t_imu, NULL, imu_producer, &n);
    pthread_create(&t_temp, NULL, temp_producer, &temp_n);

    pthread_join(t_imu, NULL);
    pthread_join(t_temp, NULL);
    pthread_join(t_con, NULL);

    sm_get_stats(&mgr, &st);

    CHECK_EQ(st.imu.pushed, (uint32_t)n);
    CHECK_EQ(st.temp.pushed, (uint32_t)temp_n);

    /* The accounting identity. Every sample is delivered, dropped, or still
     * buffered - exactly once, never twice, never nowhere. */
    CHECK_EQ(atomic_load(&g_delivered) + st.imu.dropped + st.imu.count, (uint32_t)n);
    CHECK_EQ(atomic_load(&g_temp_delivered) + st.temp.dropped + st.temp.count,
             (uint32_t)temp_n);

    /* No torn records and no sequence inversions. */
    CHECK_EQ(atomic_load(&g_incoherent), 0);
    CHECK_EQ(atomic_load(&g_seq_regressions), 0);

    sm_deinit(&mgr);
}

/*
 * Consumer deliberately starved: the phone is out of range. The buffer must
 * survive, keep only the latest 100, and report the loss exactly. This is the
 * concurrent version of the overflow requirement.
 */
TEST(overflow_under_concurrency_is_counted_exactly)
{
    pthread_t       t_imu;
    int             n = STRESS_SAMPLES;
    sm_stats_t      st;
    sm_imu_sample_t buf[SM_IMU_CAPACITY];
    size_t          got, i;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    reset_counters();

    pthread_create(&t_imu, NULL, imu_producer, &n);
    pthread_join(t_imu, NULL); /* nobody consumed anything */

    sm_get_stats(&mgr, &st);
    CHECK_EQ(st.imu.pushed, (uint32_t)n);
    CHECK_EQ(st.imu.count, SM_IMU_CAPACITY);
    CHECK_EQ(st.imu.dropped, (uint32_t)n - SM_IMU_CAPACITY);

    got = sm_read_imu(&mgr, buf, SM_IMU_CAPACITY);
    CHECK_EQ(got, SM_IMU_CAPACITY);
    for (i = 0; i < got; i++) {
        CHECK(record_is_coherent(&buf[i]));
    }
    /* Retained window is the newest, and it is contiguous. */
    CHECK_EQ(buf[0].seq, (uint32_t)n - SM_IMU_CAPACITY);
    CHECK_EQ(buf[got - 1].seq, (uint32_t)n - 1);

    sm_deinit(&mgr);
}

/*
 * Contract violation, deliberately: two threads producing into one stream.
 * The documented contract is one producer per stream, but the sequence
 * counters are atomic specifically so that a violation degrades to
 * "records may be stored slightly out of order" rather than to corruption or
 * a duplicated sequence number. This test pins that degradation.
 */
TEST(two_producers_on_one_stream_stay_memory_safe)
{
    pthread_t  a, b;
    int        n = 5000;
    sm_stats_t st;

    CHECK_EQ(sm_init(&mgr, 0), SM_OK);
    reset_counters();

    pthread_create(&a, NULL, imu_producer, &n);
    pthread_create(&b, NULL, imu_producer, &n);
    pthread_join(a, NULL);
    pthread_join(b, NULL);

    sm_get_stats(&mgr, &st);
    CHECK_EQ(st.imu.pushed, (uint32_t)(2 * n));
    CHECK_EQ(st.imu.count + st.imu.dropped, (uint32_t)(2 * n));

    /* Every retained record is still internally coherent: no interleaved
     * half-writes even with the contract broken. */
    {
        sm_imu_sample_t buf[SM_IMU_CAPACITY];
        size_t          got = sm_read_imu(&mgr, buf, SM_IMU_CAPACITY);
        size_t          i;
        for (i = 0; i < got; i++) {
            CHECK(record_is_coherent(&buf[i]));
        }
    }
    sm_deinit(&mgr);
}

TEST_MAIN_BEGIN("concurrency")
RUN(concurrent_producers_and_consumer_account_for_every_sample);
RUN(overflow_under_concurrency_is_counted_exactly);
RUN(two_producers_on_one_stream_stay_memory_safe);
TEST_MAIN_END()
