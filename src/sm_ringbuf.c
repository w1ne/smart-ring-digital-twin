/*
 * sm_ringbuf.c - implementation of the fixed-capacity overwrite-oldest buffer.
 *
 * The rationale for every design choice (no allocation, overwrite-oldest,
 * counted overflow, element-size-generic, lock-guarded) lives in sm_ringbuf.h.
 * This file is the mechanism: index arithmetic that must stay correct across
 * wraparound, and critical sections that must stay short. All wrap arithmetic
 * is modulo `capacity`, never a power-of-two mask, because the product
 * capacities (100, 20) are not powers of two.
 */
#include "sm_ringbuf.h"

#include <string.h>

/* Index of the oldest live record. Caller must hold the lock. */
static uint16_t tail_index(const sm_rb_t *rb)
{
    /* head is the next write slot; back up by count, wrapping. */
    uint16_t back = (uint16_t)(rb->count % rb->capacity);
    return (uint16_t)((rb->head + rb->capacity - back) % rb->capacity);
}

/*
 * Copy up to `max` oldest-first records into `out`. Caller must hold the lock.
 *
 * Done as at most two contiguous memcpy spans (the run up to the end of
 * storage, then the wrapped remainder) rather than one memcpy per record.
 * This matters because it runs inside the critical section, which on the
 * target masks interrupts: bulk spans keep the worst-case hold proportional
 * to bytes moved rather than to per-record call overhead, and let the
 * compiler and the Cortex-M's memcpy do the work.
 */
static size_t copy_out(const sm_rb_t *rb, void *out, size_t max)
{
    size_t   n     = rb->count < max ? rb->count : max;
    uint16_t tail  = tail_index(rb);
    uint8_t *dst   = (uint8_t *)out;
    size_t   first = rb->capacity - tail; /* records before storage wraps */

    if (first > n) {
        first = n;
    }

    memcpy(dst, rb->storage + (size_t)tail * rb->elem_size, first * rb->elem_size);

    if (n > first) {
        memcpy(dst + first * rb->elem_size, rb->storage, (n - first) * rb->elem_size);
    }

    return n;
}

int sm_rb_init(sm_rb_t *rb, void *storage, uint16_t elem_size, uint16_t capacity)
{
    if (rb == NULL || storage == NULL || elem_size == 0 || capacity == 0) {
        return -1;
    }

    rb->storage   = (uint8_t *)storage;
    rb->elem_size = elem_size;
    rb->capacity  = capacity;
    rb->head      = 0;
    rb->count     = 0;
    rb->pushed    = 0;
    rb->dropped   = 0;

    return sm_lock_init(&rb->lock);
}

void sm_rb_deinit(sm_rb_t *rb)
{
    if (rb == NULL) {
        return;
    }
    sm_lock_deinit(&rb->lock);
    rb->storage = NULL;
}

bool sm_rb_push(sm_rb_t *rb, const void *elem)
{
    bool overwrote;

    sm_lock_acquire(&rb->lock);

    memcpy(rb->storage + (size_t)rb->head * rb->elem_size, elem, rb->elem_size);
    rb->head = (uint16_t)((rb->head + 1) % rb->capacity);

    overwrote = (rb->count == rb->capacity);
    if (overwrote) {
        /* Full: the record we just landed on was unread. Tail follows head
         * implicitly because count stays pinned at capacity. */
        rb->dropped++;
    } else {
        rb->count++;
    }
    rb->pushed++;

    sm_lock_release(&rb->lock);
    return overwrote;
}

size_t sm_rb_read(sm_rb_t *rb, void *out, size_t max)
{
    size_t n;

    sm_lock_acquire(&rb->lock);
    n         = copy_out(rb, out, max);
    rb->count = (uint16_t)(rb->count - n);
    sm_lock_release(&rb->lock);

    return n;
}

size_t sm_rb_peek(const sm_rb_t *rb, void *out, size_t max)
{
    size_t n;
    /* Cast away const: the lock is mutable state, the records are not. */
    sm_rb_t *mut = (sm_rb_t *)rb;

    sm_lock_acquire(&mut->lock);
    n = copy_out(rb, out, max);
    sm_lock_release(&mut->lock);

    return n;
}

size_t sm_rb_consume(sm_rb_t *rb, size_t n)
{
    sm_lock_acquire(&rb->lock);
    if (n > rb->count) {
        n = rb->count;
    }
    rb->count = (uint16_t)(rb->count - n);
    sm_lock_release(&rb->lock);

    return n;
}

uint16_t sm_rb_count(sm_rb_t *rb)
{
    uint16_t n;

    sm_lock_acquire(&rb->lock);
    n = rb->count;
    sm_lock_release(&rb->lock);

    return n;
}

void sm_rb_stats(sm_rb_t *rb, sm_rb_stats_t *out)
{
    sm_lock_acquire(&rb->lock);
    out->count    = rb->count;
    out->capacity = rb->capacity;
    out->pushed   = rb->pushed;
    out->dropped  = rb->dropped;
    sm_lock_release(&rb->lock);
}

void sm_rb_reset(sm_rb_t *rb)
{
    sm_lock_acquire(&rb->lock);
    rb->count = 0;
    rb->head  = 0;
    sm_lock_release(&rb->lock);
}
