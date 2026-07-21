/*
 * Zephyr / nRF Connect SDK port for the nRF54L15.
 *
 * STATUS: this is the live port. It is compiled into the Zephyr app under
 * firmware/zephyr (SM_PORT_ZEPHYR) and runs in the LabWired nRF54L15 sim,
 * backing the sensor-manager core's locking and timebase with real kernel
 * primitives (k_spinlock, k_uptime_ticks).
 *
 * Why k_spinlock and not k_mutex
 * ------------------------------
 * The IMU producer is the sensor data-ready interrupt. Taking a k_mutex in
 * ISR context is illegal in Zephyr and will assert. k_spinlock is ISR-safe:
 * on a uniprocessor Cortex-M it reduces to an interrupt lock plus a
 * compiler barrier, which is precisely the primitive this buffer needs.
 *
 * The cost is that the critical section masks interrupts. That is why the
 * ring buffer keeps its sections short and bounded - the longest is the bulk
 * memcpy in sm_rb_read(), which the BLE thread controls the size of via
 * `max`. If that ever measurably perturbs radio timing (see Issue A in
 * docs/part3-debugging.md, where exactly this class of coupling is the prime
 * suspect), the fix is to cap `max` per call rather than to abandon masking.
 */
#include "../sm_port.h"

int sm_lock_init(sm_lock_t *lock)
{
    /* struct k_spinlock needs no runtime init; zeroing is sufficient. */
    lock->lock = (struct k_spinlock){0};
    return 0;
}

void sm_lock_deinit(sm_lock_t *lock)
{
    (void)lock;
}

void sm_lock_acquire(sm_lock_t *lock)
{
    lock->key = k_spin_lock(&lock->lock);
}

void sm_lock_release(sm_lock_t *lock)
{
    k_spin_unlock(&lock->lock, lock->key);
}

uint64_t sm_time_now_us(void)
{
    /*
     * k_uptime_ticks() is backed by the system timer. On the nRF54L15 that
     * should be configured to the GRTC (32.768 kHz, retained through System
     * OFF) rather than a CPU-clocked SysTick, so timestamps stay coherent
     * across the deep-sleep transitions the power strategy depends on.
     *
     * Note the resolution consequence: at 32.768 kHz, one tick is ~30.5 µs.
     * That is fine for a 20 ms IMU period but it is *not* fine as the basis
     * for PPG inter-sample timing, which is why the design puts hardware
     * timestamping at the PPG front end instead.
     */
    return (uint64_t)k_ticks_to_us_floor64(k_uptime_ticks());
}
