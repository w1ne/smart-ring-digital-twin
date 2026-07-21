/*
 * Cortex-M33 startup for the nRF54L15 application core.
 *
 * Deliberately dependency-free: no CMSIS, no vendor SDK, no libc startup. The
 * point of the onboarding smoke firmware is to exercise the *chip profile*, so
 * the less code between reset and the first UARTE byte, the more precisely a
 * failure localises to the simulator rather than to a vendor HAL.
 *
 * The vector table is placed at RRAM base (0x0) by the linker script. On
 * nRF54L the NVM is RRAM rather than flash, but from the core's point of view
 * reset behaviour is identical: load SP from word 0, PC from word 1.
 */
#include <stdint.h>

#include "board.h"
#include "nrf54l15.h"

extern int main(void);

/*
 * BLE connection-event model (WFI / idle-skippable).
 * --------------------------------------------------
 * SysTick (system exception 15) is the firmware's ONLY armed wake source. It
 * fires every SERVICE_TICK_MS and does double duty:
 *   - it wakes the main loop to service the sensors (drain / produce), and
 *   - once per CONN_INTERVAL_MS it opens a BLE connection event.
 *
 * A connection event does NOT busy-hold the CPU. The handler only RECORDS a
 * deadline (`g_busy_until = now + BLE_EVENT_WINDOW_US`); while GRTC time is
 * before that deadline the main loop WFIs WITHOUT draining. The PPG FIFO
 * therefore backs up for the whole window (CPU starvation → overflow, exactly
 * as a real link-layer stack stealing the CPU would cause) — but the window is
 * IDLE, so the simulator's event scheduler fast-forwards it instead of
 * interpreting ~3 M spin instructions per event. This is the DIGITAL/timing
 * model of the "PPG unreliable under BLE load" bug (CPU starvation + poll
 * jitter), NOT the analog RF supply-noise coupling a digital simulator cannot
 * represent.
 *
 * The window (25 ms) is shorter than the connection interval (30 ms), so each
 * interval still leaves the drain loop a live slice — contention, not a total
 * stall. Timing constants live in nrf54l15.h so main.c and this file agree.
 */
#define CONN_TICKS   (CONN_INTERVAL_MS / SERVICE_TICK_MS)

volatile uint32_t g_ble_on = 0;      /* gate: handler is a no-op until set */
volatile uint32_t g_ble_events = 0;  /* connection events serviced */
volatile uint64_t g_busy_until = 0;  /* drain parked until this GRTC us */
volatile uint32_t g_tick = 0;        /* service ticks since the pass armed */

/* Provided by the linker script. */
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

void Reset_Handler(void);

static void Default_Handler(void)
{
    /* Spin rather than reset: a simulator run that lands here stops with a
     * recognisable PC instead of looping through reset forever. */
    for (;;) {
    }
}

void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));

/*
 * SysTick ISR = the periodic service tick, and every CONN_TICKS-th tick the
 * opening of a BLE connection event. Guarded by g_ble_on so the connection-
 * event logic is inert until main() arms the BLE-load pass. The handler is
 * deliberately trivial (no busy-hold): it records the window deadline and
 * returns immediately, so it neither burns instructions nor blocks the
 * simulator's idle fast-forward. The "CPU is stolen" effect is produced by the
 * main loop electing not to drain — and WFI'ing — until g_busy_until.
 */
void SysTick_Handler(void)
{
    if (g_ble_on == 0) {
        return;
    }
    g_tick++;
    if (g_tick % CONN_TICKS == 0) {
        g_busy_until = board_time_us() + (uint64_t)BLE_EVENT_WINDOW_US;
        g_ble_events++;
    }
}

/*
 * ARMv8-M exception table. Only the architectural entries are populated; the
 * nRF54L15 has ~270 external IRQs but the smoke firmware runs entirely
 * polled, so an empty external vector region is honest rather than a
 * placeholder table that pretends handlers exist.
 */
/*
 * Entry 0 is the initial stack pointer, not a function pointer, so the table is
 * typed as a union rather than cast through a function-pointer type (which ISO
 * C forbids for object pointers, and -Wpedantic correctly rejects).
 */
typedef union {
    void (*handler)(void);
    uint32_t *stack_top;
} vector_entry_t;

__attribute__((section(".isr_vector"), used))
const vector_entry_t g_vectors[] = {
    { .stack_top = &_estack },
    { .handler = Reset_Handler },
    { .handler = NMI_Handler },
    { .handler = HardFault_Handler },
    { .handler = MemManage_Handler },
    { .handler = BusFault_Handler },
    { .handler = UsageFault_Handler },
    { .handler = 0 }, { .handler = 0 }, { .handler = 0 }, { .handler = 0 },
    { .handler = SVC_Handler },
    { .handler = DebugMon_Handler },
    { .handler = 0 },
    { .handler = PendSV_Handler },
    { .handler = SysTick_Handler },
};

void Reset_Handler(void)
{
    uint32_t *src, *dst;

    /* .data: RRAM -> RAM */
    src = &_sidata;
    for (dst = &_sdata; dst < &_edata; ) {
        *dst++ = *src++;
    }

    /* .bss: zero */
    for (dst = &_sbss; dst < &_ebss; ) {
        *dst++ = 0;
    }

    (void)main();

    for (;;) {
    }
}
