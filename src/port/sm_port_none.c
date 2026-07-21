/*
 * Single-threaded port. Locks compile to nothing.
 *
 * Used by the deterministic unit tests, which drive the core with injected
 * virtual time and no threads, so that logic failures cannot hide behind
 * scheduling noise. Concurrency is verified separately by the POSIX-port
 * stress test under TSan.
 *
 * This port is also the honest baseline for a bare-metal superloop build
 * where acquisition and consumption happen in the same context.
 */
#include "../sm_port.h"

int sm_lock_init(sm_lock_t *lock)
{
    (void)lock;
    return 0;
}

void sm_lock_deinit(sm_lock_t *lock)
{
    (void)lock;
}
void sm_lock_acquire(sm_lock_t *lock)
{
    (void)lock;
}
void sm_lock_release(sm_lock_t *lock)
{
    (void)lock;
}

/*
 * There is no clock in this port, and that is deliberate. Every core API that
 * needs a timestamp takes it as a parameter (sm_submit_*, sm_rate_poll), so
 * the deterministic tests supply virtual time directly rather than reading an
 * ambient one. A test that cannot control time cannot assert on a rate.
 */
uint64_t sm_time_now_us(void)
{
    return 0;
}
