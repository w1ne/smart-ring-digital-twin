/*
 * sensor_manager.h - simplified smart-ring sensor manager.
 *
 * Responsibilities
 * ----------------
 *   - decide *when* each sensor is due (rate control, drift-free)
 *   - accept samples from the acquisition context (ISR or sampling thread)
 *   - retain the latest N samples per stream in a thread-safe ring buffer
 *   - hand batches to the BLE subsystem on demand, with enough metadata
 *     for the phone to detect and quantify data loss
 *
 * Explicitly NOT responsibilities: talking to sensor silicon, talking to the
 * BLE stack, owning threads. Those are the caller's. That is what keeps this
 * file testable on a host and runnable on a simulator.
 *
 * Data flow
 * ---------
 *   IMU DRDY IRQ  ─┐
 *                  ├─> sm_submit_*() ─> ring buffer ─> sm_read_*() ─> GATT notify
 *   1 Hz timer    ─┘      (producer)                      (consumer)
 *
 * The producer never blocks and never fails. The consumer pulls in batches.
 * Backpressure is resolved by dropping the oldest data and *saying so*.
 */
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sm_ringbuf.h"

/* Assessment-specified retention. Sized as compile-time constants so RAM cost
 * is visible at build time: 100*28 + 20*16 = ~3.1 KB. */
#define SM_IMU_CAPACITY  100u
#define SM_TEMP_CAPACITY 20u

#define SM_IMU_RATE_HZ  50u
#define SM_TEMP_RATE_HZ 1u

/*
 * If the sampling loop is starved for longer than this many periods, do not
 * try to make up every missed sample. Resynchronise and count the loss.
 * Without this clamp, a 2-second stall at 50 Hz produces a 100-sample burst
 * that instantly overruns the buffer and evicts the good data we still had.
 */
#define SM_RATE_MAX_CATCHUP 4u

/* Spatial axes per inertial vector (x, y, z). Named so the wire encoder and
 * the record layout share one source of truth instead of a scattered `3`. */
#define SM_IMU_AXES 3

typedef enum {
    SM_OK        = 0,
    SM_ERR_ARG   = -1,
    SM_ERR_STATE = -2,
} sm_status_t;

/*
 * In-memory sample records.
 *
 * These are the storage format, deliberately not the wire format. A 64-bit
 * absolute timestamp is right for storage (no wrap, no ambiguity across
 * System OFF) and wrong for the air interface, where it would be 8 bytes of
 * near-identical data per sample. The BLE layer emits one absolute base
 * timestamp per batch plus a small per-sample delta. See docs/part1.
 */
typedef struct {
    uint32_t seq;                /* monotonic per stream; gaps == lost samples */
    uint64_t t_us;               /* acquisition time, monotonic microseconds */
    int16_t  accel[SM_IMU_AXES]; /* raw sensor LSB, x/y/z */
    int16_t  gyro[SM_IMU_AXES];  /* raw sensor LSB, x/y/z */
} sm_imu_sample_t;

typedef struct {
    uint32_t seq;
    uint64_t t_us;
    int32_t  milli_celsius; /* -40000..85000; int32 because int16 milli-degrees
                             * overflows at 32.7 C, which is inside skin range */
} sm_temp_sample_t;

/*
 * Drift-free periodic rate controller.
 *
 * next_due advances by exactly period_us per fire, so scheduling error does
 * not accumulate: 50 Hz means 50 samples in every second forever, not 49.8
 * after an hour because each period paid a few µs of latency.
 */
typedef struct {
    uint64_t period_us;
    uint64_t next_due_us;
    uint32_t fired;  /* total periods reported due */
    uint32_t missed; /* periods dropped by the catch-up clamp */
    bool     started;
} sm_rate_t;

/* Lifetime diagnostics, reported over BLE so the phone (and CI) can see
 * exactly where samples went. */
typedef struct {
    sm_rb_stats_t imu;
    sm_rb_stats_t temp;
    uint32_t      imu_missed_deadlines;
    uint32_t      temp_missed_deadlines;
} sm_stats_t;

typedef struct {
    sm_rb_t          imu_rb;
    sm_rb_t          temp_rb;
    sm_imu_sample_t  imu_store[SM_IMU_CAPACITY];
    sm_temp_sample_t temp_store[SM_TEMP_CAPACITY];
    sm_rate_t        imu_rate;
    sm_rate_t        temp_rate;
    void            *seq_impl; /* opaque: atomic sequence counters */
    bool             initialised;
} sensor_manager_t;

/* ---- rate control (standalone and separately testable) ---- */

/*
 * Arm a rate controller for `hz` firings per second, phased from `now_us`
 * (microseconds). The first period comes due at now_us + 1e6/hz. hz == 0 is
 * valid and yields a controller that never fires. Not thread-safe; a rate
 * controller belongs to its acquisition context.
 */
void sm_rate_init(sm_rate_t *r, uint32_t hz, uint64_t now_us);

/*
 * How many periods have come due at `now_us`. Returns 0 when not yet due.
 * Returns >1 only when the caller was late; capped at SM_RATE_MAX_CATCHUP,
 * with the excess accounted in `missed`.
 */
uint32_t sm_rate_poll(sm_rate_t *r, uint64_t now_us);

/* ---- lifecycle ---- */

/*
 * Initialise `mgr` in place, binding both ring buffers to the manager's own
 * inline storage and phasing the rate controllers from `now_us` (microseconds).
 * Returns SM_OK, SM_ERR_ARG on a NULL manager, or SM_ERR_STATE if a lock
 * resource could not be acquired. Must be called before any other sm_* call.
 */
sm_status_t sm_init(sensor_manager_t *mgr, uint64_t now_us);

/* Release lock resources. Idempotent; a NULL or uninitialised manager is a
 * no-op. The manager must not be used afterwards without re-init. */
void sm_deinit(sensor_manager_t *mgr);

/* ---- producer side (acquisition context) ---- */

/*
 * Submit one sample. The manager stamps the sequence number and timestamp;
 * the caller supplies only sensor data, so there is exactly one place that
 * can get sequencing wrong.
 *
 * Never blocks beyond the buffer critical section and never fails on a full
 * buffer. Returns true if this push evicted an unread sample.
 *
 * Contract: one producer per stream. The sequence counters are atomic so a
 * violation is not memory-unsafe, but records could then be stored slightly
 * out of sequence order.
 */
bool sm_submit_imu(sensor_manager_t *mgr, const int16_t accel[SM_IMU_AXES],
                   const int16_t gyro[SM_IMU_AXES], uint64_t t_us);
bool sm_submit_temp(sensor_manager_t *mgr, int32_t milli_celsius, uint64_t t_us);

/* ---- consumer side (BLE subsystem) ---- */

/*
 * Drain up to `max` oldest-first samples into `out` and remove them.
 * Returns the number of samples written. This is what the GATT notify path
 * calls once per connection event, sized to the negotiated ATT MTU.
 */
size_t sm_read_imu(sensor_manager_t *mgr, sm_imu_sample_t *out, size_t max);
size_t sm_read_temp(sensor_manager_t *mgr, sm_temp_sample_t *out, size_t max);

/*
 * Non-destructive variants, for a BLE layer that must be able to retry a
 * notification the stack refused. Peek, transmit, then sm_release_*() the
 * count that was actually accepted.
 */
size_t sm_peek_imu(sensor_manager_t *mgr, sm_imu_sample_t *out, size_t max);
size_t sm_peek_temp(sensor_manager_t *mgr, sm_temp_sample_t *out, size_t max);
size_t sm_release_imu(sensor_manager_t *mgr, size_t n);
size_t sm_release_temp(sensor_manager_t *mgr, size_t n);

/* Live count of undrained samples per stream, for deciding whether a
 * connection event is worth the radio time. Thread-safe (buffer-locked). */
uint16_t sm_pending_imu(sensor_manager_t *mgr);
uint16_t sm_pending_temp(sensor_manager_t *mgr);

/* Snapshot lifetime diagnostics into `out` (buffer accounting plus missed
 * deadlines). Thread-safe and cheap; safe to poll every connection event. */
void sm_get_stats(sensor_manager_t *mgr, sm_stats_t *out);

#endif /* SENSOR_MANAGER_H */
