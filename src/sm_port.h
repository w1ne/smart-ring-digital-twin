/*
 * sm_port.h - platform port layer for the sensor manager.
 *
 * The sensor manager core is pure C11 with no RTOS dependency. Everything
 * platform-specific lives behind these four functions and one type. Adding a
 * target means adding one file, not touching the core.
 *
 * Concurrency contract
 * --------------------
 * The core calls sm_lock_acquire()/sm_lock_release() around every buffer
 * mutation and around bulk copy-out. Critical sections are short and bounded:
 * the longest is a memcpy of at most `max` records.
 *
 * IMPORTANT for the embedded ports: on real hardware the producer is an ISR
 * (sensor data-ready interrupt or timer). A mutex cannot be taken from an ISR.
 * The correct primitive there is interrupt masking (Zephyr: k_spinlock, which
 * degenerates to irq_lock on uniprocessor; bare-metal nrfx: __disable_irq or a
 * BASEPRI raise). The POSIX port uses a mutex because on a host there is no
 * ISR context and a mutex gives us TSan coverage for free.
 */
#ifndef SM_PORT_H
#define SM_PORT_H

#include <stdint.h>

#if defined(SM_PORT_POSIX)

#include <pthread.h>
typedef pthread_mutex_t sm_lock_t;

#elif defined(SM_PORT_ZEPHYR)

#include <zephyr/kernel.h>
/* k_spinlock is ISR-safe; k_mutex is not. See note above. */
typedef struct {
    struct k_spinlock lock;
    k_spinlock_key_t  key;
} sm_lock_t;

#elif defined(SM_PORT_BAREMETAL)

/* Interrupt-masking port: stores the saved PRIMASK/BASEPRI across the section. */
typedef struct {
    uint32_t saved;
} sm_lock_t;

#elif defined(SM_PORT_NONE)

/* Single-threaded port. Locks compile to nothing. Used by deterministic
 * unit tests that want to assert on core logic without threading noise. */
typedef struct {
    uint8_t unused;
} sm_lock_t;

#else
#error                                                                                   \
    "Define exactly one of SM_PORT_POSIX / SM_PORT_ZEPHYR / SM_PORT_BAREMETAL / SM_PORT_NONE"
#endif

/* Initialise a lock. Returns 0 on success. */
int sm_lock_init(sm_lock_t *lock);

/* Release any resources held by the lock. */
void sm_lock_deinit(sm_lock_t *lock);

/* Enter/leave the critical section. Must not be nested. */
void sm_lock_acquire(sm_lock_t *lock);
void sm_lock_release(sm_lock_t *lock);

/*
 * Monotonic microsecond timestamp. Must never go backwards and must not wrap
 * within the lifetime of a session (64-bit µs is ~584,000 years).
 *
 * On the ring this maps to the nRF54L15 Global RTC / GRTC (32.768 kHz,
 * runs through System OFF), not the CPU cycle counter, so timestamps stay
 * valid across sleep.
 */
uint64_t sm_time_now_us(void);

#endif /* SM_PORT_H */
