/*
 * Sensor-manager port for the nRF54L15 (SM_PORT_BAREMETAL).
 *
 * This file is the entire platform dependency of the sensor manager core. The
 * core's .c files are compiled here UNMODIFIED, straight out of ../../src —
 * the same sources the host unit tests build. That is the claim the port layer
 * exists to make, and building both ways is what proves it rather than
 * asserting it.
 *
 * Locking: interrupt masking, not a mutex.
 * ----------------------------------------
 * On this target the producer is (or will be) the IMU data-ready ISR. A mutex
 * cannot be taken from an ISR — it deadlocks. The correct primitive is to mask
 * interrupts for the duration of the critical section, which on Cortex-M33 is
 * a PRIMASK set/restore.
 *
 * The save/restore form matters: a bare `cpsie i` on release would *enable*
 * interrupts even if the caller was already inside a masked region, silently
 * breaking a nested critical section. Saving PRIMASK and restoring it makes
 * the lock composable.
 *
 * Cost: this masks interrupts, including the radio's. That is why the ring
 * buffer keeps its critical sections short and bounded — the longest is a
 * two-span memcpy whose size the BLE layer controls via `max`. See Issue A in
 * docs/part3-debugging.md, where exactly this coupling is the prime suspect
 * for PPG corruption under BLE load.
 */
#include "sm_port.h"

#include "board.h"

int sm_lock_init(sm_lock_t *lock)
{
    lock->saved = 0;
    return 0;
}

void sm_lock_deinit(sm_lock_t *lock)
{
    (void)lock;
}

void sm_lock_acquire(sm_lock_t *lock)
{
    uint32_t primask;

    __asm volatile("mrs %0, primask" : "=r"(primask));
    __asm volatile("cpsid i" ::: "memory");
    lock->saved = primask;
}

void sm_lock_release(sm_lock_t *lock)
{
    /* Restore, do not blindly enable. */
    __asm volatile("msr primask, %0" ::"r"(lock->saved) : "memory");
}

uint64_t sm_time_now_us(void)
{
    /*
     * GRTC, not a CPU-clocked SysTick. The GRTC runs from the low-frequency
     * domain and keeps counting through the deep-sleep states the ring's power
     * strategy depends on, so timestamps stay coherent across a sleep the CPU
     * clock would not survive.
     */
    return board_time_us();
}
