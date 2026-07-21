/*
 * Concurrency stress, ported from the pthreads harness to Zephyr threads so it
 * runs under Ztest on native_sim against the real firmware port (SM_PORT_ZEPHYR,
 * k_spinlock). The producer/consumer logic and the assertions are the SAME as
 * tests/test_concurrency.c; only the threading primitives changed
 * (pthread_create -> k_thread_create, sched_yield -> k_yield).
 *
 * Two properties are checked, and they are different things:
 *
 *   1. Memory safety / no data races  - a sanitizer's job.
 *   2. Accounting integrity           - our job. Every sample that entered the
 *      manager must be accounted for exactly once as delivered, dropped, or
 *      still buffered. A ring buffer that loses a sample without incrementing
 *      `dropped` is the exact bug this test exists to catch, and it is
 *      invisible to a sanitizer.
 *
 * TSan NOTE: ThreadSanitizer instruments host pthreads, not the Zephyr kernel's
 * cooperatively/preemptively scheduled threads on native_sim, so race detection
 * is NOT preserved by this native_sim build -- it covers property (2). Property
 * (1)'s race coverage is retained by the host pthreads variant
 * (tests/test_concurrency.c built as test_concurrency_tsan via
 * `cmake -DSM_SANITIZERS=ON`). See tests/ztest/README.md.
 *
 * Records also carry an internal consistency relation so a torn read (a
 * consumer observing half of one sample and half of another) is detectable.
 */
#include <stdatomic.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "sensor_manager.h"

#define STRESS_SAMPLES 20000

/* One stack per concurrent worker. The busiest case (producer/consumer) needs
 * three live at once; the others reuse a subset. Threads are joined before the
 * next case runs, so sharing these across cases is safe. */
#define WORKER_STACK_SIZE 4096
#define WORKER_PRIO       5 /* preemptible, so producers and consumer interleave */

K_THREAD_STACK_DEFINE(stack_a, WORKER_STACK_SIZE);
K_THREAD_STACK_DEFINE(stack_b, WORKER_STACK_SIZE);
K_THREAD_STACK_DEFINE(stack_c, WORKER_STACK_SIZE);

static struct k_thread thread_a;
static struct k_thread thread_b;
static struct k_thread thread_c;

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

static void imu_producer(void *p1, void *p2, void *p3)
{
    int n = POINTER_TO_INT(p1);
    int i;

    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    for (i = 0; i < n; i++) {
        int16_t a[3], g[3];
        encode((int16_t)i, a, g);
        sm_submit_imu(&mgr, a, g, (uint64_t)i * 20000ull);
        if ((i & 0x3F) == 0) {
            k_yield(); /* widen the interleaving window */
        }
    }
    atomic_fetch_add(&g_producers_done, 1);
}

static void temp_producer(void *p1, void *p2, void *p3)
{
    int n = POINTER_TO_INT(p1);
    int i;

    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    for (i = 0; i < n; i++) {
        sm_submit_temp(&mgr, 33000 + (i % 500), (uint64_t)i * 1000000ull);
        if ((i & 0x1F) == 0) {
            k_yield();
        }
    }
    atomic_fetch_add(&g_producers_done, 1);
}

/* Drains both streams the way the BLE thread would: in bounded batches,
 * repeatedly, until the producers stop and the buffers are empty. */
static void consumer(void *p1, void *p2, void *p3)
{
    int      expected_producers = POINTER_TO_INT(p1);
    uint32_t last_seq           = 0;
    bool     have_last          = false;

    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

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
            k_yield();
        }
    }
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
ZTEST(concurrency, concurrent_producers_and_consumer_account_for_every_sample)
{
    int        n         = STRESS_SAMPLES;
    int        temp_n    = STRESS_SAMPLES / 20;
    int        producers = 2;
    sm_stats_t st;

    zassert_equal(sm_init(&mgr, 0), SM_OK);
    reset_counters();

    k_thread_create(&thread_c, stack_c, WORKER_STACK_SIZE, consumer,
                    INT_TO_POINTER(producers), NULL, NULL, WORKER_PRIO, 0, K_NO_WAIT);
    k_thread_create(&thread_a, stack_a, WORKER_STACK_SIZE, imu_producer,
                    INT_TO_POINTER(n), NULL, NULL, WORKER_PRIO, 0, K_NO_WAIT);
    k_thread_create(&thread_b, stack_b, WORKER_STACK_SIZE, temp_producer,
                    INT_TO_POINTER(temp_n), NULL, NULL, WORKER_PRIO, 0, K_NO_WAIT);

    k_thread_join(&thread_a, K_FOREVER);
    k_thread_join(&thread_b, K_FOREVER);
    k_thread_join(&thread_c, K_FOREVER);

    sm_get_stats(&mgr, &st);

    zassert_equal(st.imu.pushed, (uint32_t)n);
    zassert_equal(st.temp.pushed, (uint32_t)temp_n);

    /* The accounting identity. Every sample is delivered, dropped, or still
     * buffered - exactly once, never twice, never nowhere. */
    zassert_equal(atomic_load(&g_delivered) + st.imu.dropped + st.imu.count, (uint32_t)n);
    zassert_equal(atomic_load(&g_temp_delivered) + st.temp.dropped + st.temp.count,
                  (uint32_t)temp_n);

    /* No torn records and no sequence inversions. */
    zassert_equal(atomic_load(&g_incoherent), 0);
    zassert_equal(atomic_load(&g_seq_regressions), 0);

    sm_deinit(&mgr);
}

/*
 * Consumer deliberately starved: the phone is out of range. The buffer must
 * survive, keep only the latest 100, and report the loss exactly. This is the
 * concurrent version of the overflow requirement.
 */
ZTEST(concurrency, overflow_under_concurrency_is_counted_exactly)
{
    int             n = STRESS_SAMPLES;
    sm_stats_t      st;
    sm_imu_sample_t buf[SM_IMU_CAPACITY];
    size_t          got, i;

    zassert_equal(sm_init(&mgr, 0), SM_OK);
    reset_counters();

    k_thread_create(&thread_a, stack_a, WORKER_STACK_SIZE, imu_producer,
                    INT_TO_POINTER(n), NULL, NULL, WORKER_PRIO, 0, K_NO_WAIT);
    k_thread_join(&thread_a, K_FOREVER); /* nobody consumed anything */

    sm_get_stats(&mgr, &st);
    zassert_equal(st.imu.pushed, (uint32_t)n);
    zassert_equal(st.imu.count, SM_IMU_CAPACITY);
    zassert_equal(st.imu.dropped, (uint32_t)n - SM_IMU_CAPACITY);

    got = sm_read_imu(&mgr, buf, SM_IMU_CAPACITY);
    zassert_equal(got, SM_IMU_CAPACITY);
    for (i = 0; i < got; i++) {
        zassert_true(record_is_coherent(&buf[i]));
    }
    /* Retained window is the newest, and it is contiguous. */
    zassert_equal(buf[0].seq, (uint32_t)n - SM_IMU_CAPACITY);
    zassert_equal(buf[got - 1].seq, (uint32_t)n - 1);

    sm_deinit(&mgr);
}

/*
 * Contract violation, deliberately: two threads producing into one stream.
 * The documented contract is one producer per stream, but the sequence
 * counters are atomic specifically so that a violation degrades to
 * "records may be stored slightly out of order" rather than to corruption or
 * a duplicated sequence number. This test pins that degradation.
 */
ZTEST(concurrency, two_producers_on_one_stream_stay_memory_safe)
{
    int        n = 5000;
    sm_stats_t st;

    zassert_equal(sm_init(&mgr, 0), SM_OK);
    reset_counters();

    k_thread_create(&thread_a, stack_a, WORKER_STACK_SIZE, imu_producer,
                    INT_TO_POINTER(n), NULL, NULL, WORKER_PRIO, 0, K_NO_WAIT);
    k_thread_create(&thread_b, stack_b, WORKER_STACK_SIZE, imu_producer,
                    INT_TO_POINTER(n), NULL, NULL, WORKER_PRIO, 0, K_NO_WAIT);
    k_thread_join(&thread_a, K_FOREVER);
    k_thread_join(&thread_b, K_FOREVER);

    sm_get_stats(&mgr, &st);
    zassert_equal(st.imu.pushed, (uint32_t)(2 * n));
    zassert_equal(st.imu.count + st.imu.dropped, (uint32_t)(2 * n));

    /* Every retained record is still internally coherent: no interleaved
     * half-writes even with the contract broken. */
    {
        sm_imu_sample_t buf[SM_IMU_CAPACITY];
        size_t          got = sm_read_imu(&mgr, buf, SM_IMU_CAPACITY);
        size_t          i;
        for (i = 0; i < got; i++) {
            zassert_true(record_is_coherent(&buf[i]));
        }
    }
    sm_deinit(&mgr);
}

ZTEST_SUITE(concurrency, NULL, NULL, NULL, NULL, NULL);
