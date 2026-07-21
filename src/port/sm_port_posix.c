/*
 * POSIX port. Used by the host unit tests and the desktop demo.
 *
 * A mutex is the right choice here specifically because there is no ISR
 * context on a host, and because a real mutex is what ThreadSanitizer knows
 * how to reason about - the concurrency tests get real happens-before
 * verification rather than a hand-rolled primitive TSan cannot see through.
 */
#include "../sm_port.h"

#include <errno.h>
#include <time.h>

int sm_lock_init(sm_lock_t *lock)
{
    return pthread_mutex_init(lock, NULL) == 0 ? 0 : -1;
}

void sm_lock_deinit(sm_lock_t *lock)
{
    (void)pthread_mutex_destroy(lock);
}

void sm_lock_acquire(sm_lock_t *lock)
{
    (void)pthread_mutex_lock(lock);
}

void sm_lock_release(sm_lock_t *lock)
{
    (void)pthread_mutex_unlock(lock);
}

uint64_t sm_time_now_us(void)
{
    struct timespec ts;

    /* CLOCK_MONOTONIC, not CLOCK_REALTIME: an NTP step must not make a
     * sample timestamp travel backwards. Same reasoning as using the GRTC
     * rather than a wall clock on the target. */
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0;
    }
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}
