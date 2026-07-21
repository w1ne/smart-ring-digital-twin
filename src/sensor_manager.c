/*
 * sensor_manager.c - orchestrates rate control, sequencing, retention and
 * loss accounting for the smart-ring sensor streams.
 *
 * This file owns the *policy* (drift-free scheduling, one-place sequence
 * stamping, per-stream retention) and delegates the *mechanism* to sm_ringbuf
 * (storage) and the port layer (locking, time). The IMU and temperature paths
 * are intentionally kept as thin, separately-typed wrappers over the generic
 * ring buffer rather than macro-fused: the shared logic already lives once in
 * sm_ringbuf, so the wrappers carry no duplicated behaviour to factor out.
 */
#include "sensor_manager.h"

#include <stdatomic.h>
#include <string.h>

/* Microseconds per second: converts a rate in Hz to an integer period. */
#define SM_US_PER_SEC 1000000ull

/*
 * Sequence counters live behind an opaque pointer so <stdatomic.h> stays out
 * of the public header. Embedded projects routinely mix compilers and C
 * dialects between the application and its consumers; leaking _Atomic into an
 * exported struct definition is a portability trap not worth the convenience.
 */
typedef struct {
    atomic_uint_least32_t imu_seq;
    atomic_uint_least32_t temp_seq;
} sm_seq_t;

static sm_seq_t g_seq[1]; /* one manager instance; see note in sm_init */

/* ------------------------------------------------------------------ */
/* Rate control                                                        */
/* ------------------------------------------------------------------ */

void sm_rate_init(sm_rate_t *r, uint32_t hz, uint64_t now_us)
{
    r->period_us   = (hz == 0) ? 0 : (SM_US_PER_SEC / hz);
    r->next_due_us = now_us + r->period_us;
    r->fired       = 0;
    r->missed      = 0;
    r->started     = true;
}

uint32_t sm_rate_poll(sm_rate_t *r, uint64_t now_us)
{
    uint32_t due = 0;

    if (!r->started || r->period_us == 0 || now_us < r->next_due_us) {
        return 0;
    }

    /* Count whole periods elapsed since the deadline we are servicing. */
    due = 1u + (uint32_t)((now_us - r->next_due_us) / r->period_us);

    if (due > SM_RATE_MAX_CATCHUP) {
        /*
         * We were starved. Do not emit a burst of stale samples that would
         * evict the recent data still in the buffer. Drop the backlog,
         * record it, and resynchronise the phase to now.
         */
        r->missed += due - SM_RATE_MAX_CATCHUP;
        due            = SM_RATE_MAX_CATCHUP;
        r->next_due_us = now_us + r->period_us;
    } else {
        /* Advance by exact periods: no accumulated drift. */
        r->next_due_us += (uint64_t)due * r->period_us;
    }

    r->fired += due;
    return due;
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

sm_status_t sm_init(sensor_manager_t *mgr, uint64_t now_us)
{
    if (mgr == NULL) {
        return SM_ERR_ARG;
    }

    memset(mgr, 0, sizeof(*mgr));

    if (sm_rb_init(&mgr->imu_rb, mgr->imu_store, (uint16_t)sizeof(sm_imu_sample_t),
                   SM_IMU_CAPACITY) != 0) {
        return SM_ERR_STATE;
    }
    if (sm_rb_init(&mgr->temp_rb, mgr->temp_store, (uint16_t)sizeof(sm_temp_sample_t),
                   SM_TEMP_CAPACITY) != 0) {
        sm_rb_deinit(&mgr->imu_rb);
        return SM_ERR_STATE;
    }

    sm_rate_init(&mgr->imu_rate, SM_IMU_RATE_HZ, now_us);
    sm_rate_init(&mgr->temp_rate, SM_TEMP_RATE_HZ, now_us);

    /*
     * The product has exactly one sensor manager, so a single static sequence
     * block is sufficient and keeps the public struct free of atomics. Tests
     * that instantiate several managers share these counters; that is
     * harmless because sequence numbers only need to be monotonic, not dense.
     * If multiple independent instances ever become a real requirement, this
     * becomes a per-instance allocation.
     */
    atomic_store(&g_seq->imu_seq, 0u);
    atomic_store(&g_seq->temp_seq, 0u);
    mgr->seq_impl = g_seq;

    mgr->initialised = true;
    return SM_OK;
}

void sm_deinit(sensor_manager_t *mgr)
{
    if (mgr == NULL || !mgr->initialised) {
        return;
    }
    sm_rb_deinit(&mgr->imu_rb);
    sm_rb_deinit(&mgr->temp_rb);
    mgr->initialised = false;
}

/* ------------------------------------------------------------------ */
/* Producer                                                            */
/* ------------------------------------------------------------------ */

bool sm_submit_imu(sensor_manager_t *mgr, const int16_t accel[SM_IMU_AXES],
                   const int16_t gyro[SM_IMU_AXES], uint64_t t_us)
{
    sm_seq_t       *seq = (sm_seq_t *)mgr->seq_impl;
    sm_imu_sample_t s;

    s.seq  = (uint32_t)atomic_fetch_add(&seq->imu_seq, 1u);
    s.t_us = t_us;
    memcpy(s.accel, accel, sizeof(s.accel));
    memcpy(s.gyro, gyro, sizeof(s.gyro));

    return sm_rb_push(&mgr->imu_rb, &s);
}

bool sm_submit_temp(sensor_manager_t *mgr, int32_t milli_celsius, uint64_t t_us)
{
    sm_seq_t        *seq = (sm_seq_t *)mgr->seq_impl;
    sm_temp_sample_t s;

    s.seq           = (uint32_t)atomic_fetch_add(&seq->temp_seq, 1u);
    s.t_us          = t_us;
    s.milli_celsius = milli_celsius;

    return sm_rb_push(&mgr->temp_rb, &s);
}

/* ------------------------------------------------------------------ */
/* Consumer                                                            */
/* ------------------------------------------------------------------ */

size_t sm_read_imu(sensor_manager_t *mgr, sm_imu_sample_t *out, size_t max)
{
    return sm_rb_read(&mgr->imu_rb, out, max);
}

size_t sm_read_temp(sensor_manager_t *mgr, sm_temp_sample_t *out, size_t max)
{
    return sm_rb_read(&mgr->temp_rb, out, max);
}

size_t sm_peek_imu(sensor_manager_t *mgr, sm_imu_sample_t *out, size_t max)
{
    return sm_rb_peek(&mgr->imu_rb, out, max);
}

size_t sm_peek_temp(sensor_manager_t *mgr, sm_temp_sample_t *out, size_t max)
{
    return sm_rb_peek(&mgr->temp_rb, out, max);
}

size_t sm_release_imu(sensor_manager_t *mgr, size_t n)
{
    return sm_rb_consume(&mgr->imu_rb, n);
}

size_t sm_release_temp(sensor_manager_t *mgr, size_t n)
{
    return sm_rb_consume(&mgr->temp_rb, n);
}

uint16_t sm_pending_imu(sensor_manager_t *mgr)
{
    return sm_rb_count(&mgr->imu_rb);
}

uint16_t sm_pending_temp(sensor_manager_t *mgr)
{
    return sm_rb_count(&mgr->temp_rb);
}

void sm_get_stats(sensor_manager_t *mgr, sm_stats_t *out)
{
    sm_rb_stats(&mgr->imu_rb, &out->imu);
    sm_rb_stats(&mgr->temp_rb, &out->temp);
    out->imu_missed_deadlines  = mgr->imu_rate.missed;
    out->temp_missed_deadlines = mgr->temp_rate.missed;
}
