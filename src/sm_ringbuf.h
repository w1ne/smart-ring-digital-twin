/*
 * sm_ringbuf.h - fixed-capacity, thread-safe, overwrite-oldest record buffer.
 *
 * Design decisions and why
 * ------------------------
 * 1. No dynamic allocation. Storage is supplied by the caller as a static
 *    array. On a 256 KB-RAM part with a multi-week uptime target, malloc is a
 *    fragmentation and worst-case-latency liability with no upside.
 *
 * 2. Overwrite-oldest on overflow, not reject-newest. The requirement is
 *    "retain the latest N samples". For a wearable that is also the correct
 *    product behaviour: if the phone is out of range, the most recent motion
 *    and temperature data is what matters. Dropping the newest sample to
 *    preserve a stale one is the wrong trade.
 *
 * 3. Overflow is counted, not silent. `dropped` is monotonic and is reported
 *    over BLE. Combined with the per-record sequence number this lets the
 *    mobile app say exactly how many samples it lost and where, instead of
 *    silently reconstructing a wrong timeline. Graceful != invisible.
 *
 * 4. Element-size-generic rather than macro-templated. One implementation,
 *    one set of tests, one place for the wraparound bug to not be. The cost
 *    is a memcpy instead of a struct assignment; at 50 Hz that is noise.
 *
 * 5. Lock-guarded rather than lock-free SPSC. A lock-free SPSC ring is
 *    tempting and would be correct for one producer and one consumer, but
 *    overwrite-oldest requires the producer to move the read index, which
 *    races the consumer. Rather than ship a subtly-wrong lock-free buffer,
 *    this uses a bounded critical section which on the target is an
 *    interrupt mask of a few dozen cycles. Revisit only if measurement shows
 *    the masking hurts radio timing.
 */
#ifndef SM_RINGBUF_H
#define SM_RINGBUF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sm_port.h"

typedef struct {
    uint8_t  *storage; /* caller-owned, capacity * elem_size bytes */
    uint16_t  elem_size;
    uint16_t  capacity;
    uint16_t  head;    /* index of next write */
    uint16_t  count;   /* live records, <= capacity */
    uint32_t  pushed;  /* total records ever accepted (monotonic) */
    uint32_t  dropped; /* total records overwritten before being read */
    sm_lock_t lock;
} sm_rb_t;

/* Snapshot of buffer accounting. Cheap; safe to poll. */
typedef struct {
    uint16_t count;
    uint16_t capacity;
    uint32_t pushed;
    uint32_t dropped;
} sm_rb_stats_t;

/*
 * Bind a buffer to caller-supplied storage. `storage` must be at least
 * capacity * elem_size bytes and outlive the buffer. Returns 0 on success,
 * -1 on invalid arguments.
 */
int sm_rb_init(sm_rb_t *rb, void *storage, uint16_t elem_size, uint16_t capacity);

/* Release the lock resource. The buffer must not be used afterwards. */
void sm_rb_deinit(sm_rb_t *rb);

/*
 * Append one record, copying elem_size bytes from `elem`.
 * Never fails and never blocks on a full buffer: when full, the oldest record
 * is overwritten and `dropped` is incremented.
 * Returns true if a record was overwritten (i.e. data was lost).
 */
bool sm_rb_push(sm_rb_t *rb, const void *elem);

/*
 * Copy out up to `max` records, oldest first, and remove them.
 * This is the BLE consumer's entry point: it drains in bulk so the critical
 * section is amortised over a whole notification payload rather than paid
 * per sample.
 * Returns the number of records written to `out`.
 */
size_t sm_rb_read(sm_rb_t *rb, void *out, size_t max);

/*
 * Same as sm_rb_read() but non-destructive. Used when the BLE layer wants to
 * build a notification it might have to retry: peek, transmit, and only
 * sm_rb_consume() once the stack has acknowledged the buffer.
 */
size_t sm_rb_peek(const sm_rb_t *rb, void *out, size_t max);

/* Discard the oldest `n` records. Returns the number actually discarded. */
size_t sm_rb_consume(sm_rb_t *rb, size_t n);

/* Number of live records. */
uint16_t sm_rb_count(sm_rb_t *rb);

/* Accounting snapshot. */
void sm_rb_stats(sm_rb_t *rb, sm_rb_stats_t *out);

/* Drop all records. Counters (pushed/dropped) are preserved: they are
 * lifetime diagnostics, not buffer state. */
void sm_rb_reset(sm_rb_t *rb);

#endif /* SM_RINGBUF_H */
