/*
 * nRF54L15 register subset for the smart-ring firmware.
 *
 * Secure alias (0x5000_0000 window). Addresses cross-checked against the Zephyr
 * devicetree `dts/vendor/nordic/nrf54l_05_10_15.dtsi`.
 *
 * Hand-written rather than generated CMSIS: a 60-line audited subset is easier
 * to check against the simulator's chip profile than a 20,000-line header, and
 * every address here is one this firmware actually touches.
 *
 * EVERY offset below comes from the Nordic MDK SVD / headers, NOT from nRF52.
 * The first version of this file used nRF52 offsets and appeared to work,
 * because the simulator had nRF52 models mapped: firmware and model agreed with
 * each other and both disagreed with silicon, so the boot proved nothing. Real
 * Zephyr, which uses the real offsets, hung immediately. If you add a register,
 * take it from the MDK — not from a working nRF52 example.
 */
#ifndef NRF54L15_H
#define NRF54L15_H

#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

/* ── UARTE20 — console (DT uart20 @ 0xc6000) ─────────────────────────────── */
#define UARTE20_BASE              0x500C6000UL
/* nRF54L generation: EasyDMA lives in a DMA.{RX,TX} cluster. Offsets from the
 * MDK SVD, NOT from nRF52 — see the file header. */
#define UARTE_TASKS_DMA_TX_START(b) REG32((b) + 0x050)
#define UARTE_TASKS_DMA_TX_STOP(b)  REG32((b) + 0x054)
#define UARTE_EVENTS_TXSTOPPED(b)   REG32((b) + 0x130)
#define UARTE_EVENTS_DMA_TX_END(b)  REG32((b) + 0x168)
#define UARTE_ENABLE(b)             REG32((b) + 0x500)
#define UARTE_PSEL_TXD(b)           REG32((b) + 0x604)
#define UARTE_PSEL_RXD(b)           REG32((b) + 0x60C)
#define UARTE_BAUDRATE(b)           REG32((b) + 0x524)
#define UARTE_DMA_TX_PTR(b)         REG32((b) + 0x73C)
#define UARTE_DMA_TX_MAXCNT(b)      REG32((b) + 0x740)
#define UARTE_ENABLE_UARTE        8u
#define UARTE_BAUD_115200         0x01D7E000u

/* ── TWIM21 — sensor I²C bus (DT twi21 @ 0xc7000) ────────────────────────── */
#define TWIM21_BASE               0x500C7000UL
/* Same DMA-cluster change as the UARTE. */
#define TWIM_TASKS_STOP(b)          REG32((b) + 0x004)
#define TWIM_TASKS_DMA_RX_START(b)  REG32((b) + 0x028)
#define TWIM_TASKS_DMA_TX_START(b)  REG32((b) + 0x050)
#define TWIM_EVENTS_STOPPED(b)      REG32((b) + 0x104)
#define TWIM_EVENTS_ERROR(b)        REG32((b) + 0x114)
#define TWIM_EVENTS_DMA_RX_END(b)   REG32((b) + 0x14C)
#define TWIM_EVENTS_DMA_TX_END(b)   REG32((b) + 0x168)
#define TWIM_SHORTS(b)              REG32((b) + 0x200)
#define TWIM_ERRORSRC(b)            REG32((b) + 0x4C4)
#define TWIM_ENABLE(b)              REG32((b) + 0x500)
#define TWIM_FREQUENCY(b)           REG32((b) + 0x524)
#define TWIM_ADDRESS(b)             REG32((b) + 0x588)
#define TWIM_DMA_RX_PTR(b)          REG32((b) + 0x704)
#define TWIM_DMA_RX_MAXCNT(b)       REG32((b) + 0x708)
#define TWIM_DMA_TX_PTR(b)          REG32((b) + 0x73C)
#define TWIM_DMA_TX_MAXCNT(b)       REG32((b) + 0x740)
#define TWIM_ENABLE_TWIM          6u
#define TWIM_FREQ_400K            0x06400000u

/* ── GRTC — Global RTC, the family's timekeeper (DT grtc @ 0xe2000) ──────── */
#define GRTC_BASE                 0x500E2000UL
#define GRTC_TASKS_START(b)       REG32((b) + 0x060)
#define GRTC_TASKS_CLEAR(b)       REG32((b) + 0x068)
#define GRTC_MODE(b)              REG32((b) + 0x510)
#define GRTC_SYSCOUNTERL(b)       REG32((b) + 0x720)
#define GRTC_SYSCOUNTERH(b)       REG32((b) + 0x724)
/*
 * nrfx enables the counter via MODE.SYSCOUNTEREN, not TASKS_START — its
 * nrf_grtc_task_trigger helpers actively assert against NRF_GRTC_TASK_START.
 * Both are written here so this firmware works against either interpretation.
 */
/* MODE.SYSCOUNTEREN is bit 1, not bit 0 (MDK CLOCK/GRTC field positions). */
#define GRTC_MODE_SYSCOUNTEREN    (1u << 1)

/* ── TEMP — on-die temperature sensor (DT temp @ 0xd7000) ────────────────── */
#define TEMP_BASE                 0x500D7000UL
#define TEMP_TASKS_START(b)       REG32((b) + 0x000)
#define TEMP_TASKS_STOP(b)        REG32((b) + 0x004)
#define TEMP_EVENTS_DATARDY(b)    REG32((b) + 0x100)
#define TEMP_RESULT(b)            REG32((b) + 0x508)

/* ── GPIO ────────────────────────────────────────────────────────────────
 * MDK NRF_P1_S_BASE, and on THIS family NRF_GPIO_Type puts OUT at +0x000 with
 * no leading reserved words. Nordic changed that prefix every generation —
 * nRF52840 OUT at +0x504, nRF5340 at +0x004, nRF54L15 at +0x000 — so an offset
 * copied from another Nordic part is wrong here even though the names match.
 */
#define GPIO_P1_BASE              0x500D8200UL
#define GPIO_OUT(b)               REG32((b) + 0x000)
#define GPIO_OUTSET(b)            REG32((b) + 0x004)
#define GPIO_OUTCLR(b)            REG32((b) + 0x008)
/* IN (read the live pin state) lands at +0x00C from this firmware-visible
 * base, the same struct-relative slot the simulator's GPIO model serves. The
 * 1-channel touch sensor's output is wired to P1.13, so bit 13 is the press. */
#define GPIO_IN(b)                REG32((b) + 0x00C)
#define GPIO_DIRSET(b)            REG32((b) + 0x014)
#define MOTOR_ENABLE_PIN          11u
#define TOUCH_PIN                 13u

/* PSEL: bit31 = disconnected, bits 5..6 = port, bits 0..4 = pin. */
#define PSEL(port, pin)           ((uint32_t)(((port) << 5) | (pin)))

/* ── SysTick — ARMv8-M core timer (SCS block @ 0xE000_E010) ───────────────
 * A system exception (15), not an NVIC line, so it needs no external vector
 * table. The firmware models a BLE connection event as a SysTick ISR: the
 * periodic exception whose handler holds the CPU for a connection-event
 * window, starving the PPG drain loop. Core clock is 128 MHz, so a reload for
 * an interval of T_ms is T_ms * 128000 - 1 (must fit 24 bits, <= 0xFFFFFF). */
#define SYST_CSR                  REG32(0xE000E010UL)
#define SYST_RVR                  REG32(0xE000E014UL)
#define SYST_CVR                  REG32(0xE000E018UL)
#define SYST_CSR_ENABLE           (1u << 0)
#define SYST_CSR_TICKINT          (1u << 1)
#define SYST_CSR_CLKSOURCE        (1u << 2)   /* processor clock */
#define CORE_CLOCK_HZ             128000000u

/* ── Sleep / BLE-contention timing (shared by main.c and startup.c) ────────
 * The firmware is WFI-driven: SysTick is the ONLY armed wake source, firing
 * every SERVICE_TICK_MS. Between ticks the CPU sleeps, so the simulator's
 * event scheduler fast-forwards the idle gap instead of interpreting millions
 * of spin instructions. A BLE connection event opens once per CONN_INTERVAL_MS
 * and holds the drain loop off for BLE_EVENT_WINDOW_US of sim-time — but as
 * IDLE (WFI), not a busy-spin, so it too is fast-forwarded.
 * SERVICE_TICK_MS must divide CONN_INTERVAL_MS and be < the ~20 ms it takes
 * the 1600 Hz PPG to fill its 32-deep FIFO (else pass 1 would overflow). */
#define SERVICE_TICK_MS           5u
#define CONN_INTERVAL_MS          30u
#define BLE_EVENT_WINDOW_US       25000u   /* CPU unavailable to the drain loop */

/* ── I²C addresses (smart-ring.yaml) ─────────────────────────────────────── */
#define ADDR_IMU                  0x68u   /* BMI270   */
#define ADDR_PPG                  0x57u   /* MAX30102 */
#define ADDR_SKIN_TEMP            0x48u   /* TMP117   */
#define ADDR_HAPTIC               0x5Au   /* DRV2605  */

#endif /* NRF54L15_H */
