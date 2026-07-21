/*
 * Smart-ring firmware: the Part 2 sensor manager running on real (simulated)
 * nRF54L15 silicon with a realistic sensor BOM, plus a Part 3 reproduction of
 * the "PPG unreliable under BLE load" bug — modelled at the DIGITAL/timing
 * level.
 *
 * The core sources ../../src/sensor_manager.c, ../../src/sm_ringbuf.c and
 * ../../src/ble_batch.c are compiled here UNMODIFIED — byte-for-byte the
 * sources the host unit tests build. Only the port (sm_port_nrf54l15.c) and the
 * drivers (board.c) are target-specific.
 *
 * ── BOM ───────────────────────────────────────────────────────────────────
 *   imu    BMI270  (0x68)  hardware step counter, int16 LE accel/gyro
 *   ppg    MAX30102 (0x57) 32-deep FIFO, wall-clock driven
 *   temp   TMP117  (0x48)  on-body skin temperature, 16-bit big-endian
 *   haptic DRV2605 (0x5A)  vibration motor driver
 *   touch  1-channel GPIO cap sensor on P1.13 (not I²C)
 *
 * ── The bug (Part 3) ───────────────────────────────────────────────────────
 * A BLE connection event is a periodic SysTick ISR that busy-holds the CPU for
 * a connection-event window (see startup.c). While the CPU is stuck in that
 * ISR it cannot drain the PPG FIFO. The nRF54L TWIM advances each I²C slave's
 * sample clock by the real wall-clock elapsed since that slave's last
 * transaction, so the first PPG poll after a stall advances the whole stalled
 * interval at once — and the 32-deep FIFO overflows (OVF_COUNTER, reg 0x05),
 * exactly as silicon would.
 *
 * The firmware runs the SAME sensor loop twice:
 *   pass 1  BLE off  — the CPU is always free to drain; FIFO stays empty; 0 OVF
 *   pass 2  BLE on   — SysTick steals contiguous CPU; drains fall behind; OVF>0
 *
 * This is the timing class of the fault (CPU starvation + poll jitter), NOT the
 * analog supply-noise coupling from radio TX current, which a digital simulator
 * cannot represent.
 */
#include <stdint.h>

#include "ble_batch.h"
#include "board.h"
#include "nrf54l15.h"
#include "sensor_manager.h"

/* Shared with the SysTick ISR in startup.c. */
extern volatile uint32_t g_ble_on;
extern volatile uint32_t g_ble_events;
extern volatile uint64_t g_busy_until;   /* drain parked until this GRTC us */
extern volatile uint32_t g_tick;         /* service ticks since pass armed */

#define RUN_DURATION_US    600000ull   /* 0.6 simulated seconds per pass */

/* Stall/recovery phase (Part 2 "handle overflow gracefully, retain latest 100").
 * A 50 Hz IMU producer runs throughout; the consumer drains during BASELINE and
 * RECOVERY but goes silent for STALL (phone out of range). At 50 Hz the 100-deep
 * software ring fills in 2.0 s, so STALL > 2.0 s guarantees overwrite-oldest
 * drops. Times are sim-time; the loop still WFIs between ticks. */
#define STALL_BASELINE_US  300000ull    /* 0.3 s: drained, dropped stays 0     */
#define STALL_WINDOW_US    2400000ull   /* 2.4 s: consumer silent → overflow   */
#define STALL_RECOVERY_US  400000ull    /* 0.4 s: resume draining, report gap  */
#define STALL_TOTAL_US     (STALL_BASELINE_US + STALL_WINDOW_US + STALL_RECOVERY_US)

/* Sleep until the next SysTick (or any wake event). WFI is what lets the
 * simulator fast-forward the idle gap: the CPU parks instead of spinning. */
static inline void cpu_sleep(void)
{
    __asm__ volatile("wfi");
}

static sensor_manager_t mgr;

static void print_kv(const char *k, int32_t v)
{
    board_puts(k);
    board_put_i32(v);
    board_puts("\r\n");
}

typedef struct {
    uint32_t imu_pushed;
    uint32_t ppg_drained;
    uint32_t frames;
    uint8_t  ovf;
    uint32_t events;
} pass_result_t;

typedef struct {
    uint32_t pushed;      /* samples the 50 Hz producer submitted           */
    uint32_t delivered;   /* samples the consumer released to the BLE layer */
    uint32_t dropped;     /* overwritten-oldest during the stall (from sm)  */
    uint16_t buffered;    /* live count after resume (pins at capacity)     */
    uint16_t capacity;    /* SM_IMU_CAPACITY (100)                          */
    uint32_t seq_gap;     /* lost samples inferred from sequence numbers    */
} stall_result_t;

/*
 * Phase 0 — graceful SOFTWARE ring-buffer overflow + recovery ("phone out of
 * range"). The 50 Hz IMU producer runs the whole time; the consumer drains
 * during a short baseline, goes SILENT for a ~2.4 s stall (the ring fills to its
 * 100-sample capacity and then overwrites oldest — `dropped` climbs, `count`
 * pins at 100, newest retained), then RESUMES. The first post-resume batch
 * reports the exact sequence gap so nothing is lost silently. Every number here
 * is read from the real sensor-manager stats / sample sequence numbers.
 *
 * WFI-friendly: the "stall" is simply the consumer electing not to drain; the
 * CPU still sleeps between 50 Hz service ticks.
 */
static void run_stall_recovery(stall_result_t *res)
{
    uint64_t start, now;
    uint32_t last_seq = 0;          /* seq of last sample delivered pre-stall */
    int      have_last = 0;         /* a pre-stall sample has been delivered  */
    int      was_stalled = 0;       /* previous iteration was inside the stall */
    int      resume_pending = 0;    /* armed at the resume edge to grab the gap */
    int      resume_done = 0;       /* the resume gap has been recorded        */
    uint16_t peak_buffered = 0;     /* live count sampled at the stall's end    */
    uint32_t delivered = 0;

    (void)sm_init(&mgr, board_time_us());

    /* Periodic service tick, no BLE. */
    SYST_RVR = SERVICE_TICK_MS * (CORE_CLOCK_HZ / 1000u) - 1u;
    SYST_CVR = 0;
    g_tick       = 0;
    g_busy_until = 0;
    g_ble_on     = 0;
    SYST_CSR = SYST_CSR_ENABLE | SYST_CSR_TICKINT | SYST_CSR_CLKSOURCE;

    start = board_time_us();
    for (;;) {
        uint32_t due;
        int      in_stall;

        now = board_time_us();
        if (now - start >= STALL_TOTAL_US) {
            break;
        }

        /* ---- producer: 50 Hz IMU, runs throughout (incl. the stall) ---- */
        due = sm_rate_poll(&mgr.imu_rate, now);
        while (due--) {
            int16_t accel[3], gyro[3];

            if (board_imu_read(accel, gyro) == 0) {
                sm_submit_imu(&mgr, accel, gyro, now);
            }
        }

        /* ---- consumer: drain EXCEPT during the stall window ---- */
        in_stall = ((now - start) >= STALL_BASELINE_US &&
                    (now - start) <  STALL_BASELINE_US + STALL_WINDOW_US);

        /* Stall → resume edge: sample the ring at its fullest (pins at capacity,
         * newest retained) BEFORE we start draining it down, and arm the
         * one-shot gap capture so it fires on THIS resume, not a baseline drain. */
        if (was_stalled && !in_stall && !resume_done) {
            peak_buffered  = sm_pending_imu(&mgr);
            resume_pending = 1;
        }
        was_stalled = in_stall;

        if (!in_stall && sm_pending_imu(&mgr) > 0) {
            sm_imu_sample_t batch[64];
            size_t          n = sm_peek_imu(&mgr, batch, 64);

            if (n > 0) {
                /* First drain after the stall: batch[0] is the oldest sample
                 * that SURVIVED overwrite-oldest; the gap from the last sample
                 * we delivered pre-stall is exactly what was lost. */
                if (resume_pending && have_last) {
                    res->seq_gap   = batch[0].seq - last_seq - 1u;
                    resume_pending = 0;
                    resume_done    = 1;
                }
                (void)sm_release_imu(&mgr, n);
                delivered += (uint32_t)n;
                last_seq   = batch[n - 1].seq;
                have_last  = 1;
            }
        }

        cpu_sleep();
    }

    SYST_CSR = 0;

    {
        sm_stats_t st;
        sm_get_stats(&mgr, &st);
        res->pushed    = st.imu.pushed;
        res->dropped   = st.imu.dropped;
        res->capacity  = st.imu.capacity;
    }
    res->delivered = delivered;
    res->buffered  = peak_buffered;   /* the fullest the ring got (==capacity) */
    if (!resume_done) {
        res->seq_gap = 0;
    }
}

/*
 * One sensor-manager pass. Continuously drains the PPG (keeping up is exactly
 * the property under test), runs the 50 Hz IMU / 1 Hz temp producers and the
 * BLE batch encoder consumer, for RUN_DURATION_US of simulated time.
 */
static void run_pass(int ble_on, pass_result_t *res)
{
    uint64_t start, now;
    uint32_t ppg_drained = 0, frames = 0;

    /* Clean slate: empty the FIFO and re-init the manager so each pass's stats
     * and overflow count stand alone. */
    board_ppg_reset_fifo();
    (void)sm_init(&mgr, board_time_us());

    /* Arm SysTick as the periodic service tick in BOTH passes: ENABLE |
     * TICKINT | processor clock. reload = T_ms * (core_clk / 1000) - 1, fits
     * 24 bits. The CPU sleeps (WFI) between ticks, so the simulator fast-
     * forwards the idle gaps; without a periodic wake there would be nothing
     * to fast-forward TO. */
    SYST_RVR = SERVICE_TICK_MS * (CORE_CLOCK_HZ / 1000u) - 1u;
    SYST_CVR = 0;
    g_tick     = 0;
    g_busy_until = 0;
    g_ble_on   = ble_on ? 1u : 0u;   /* gates the connection-event logic */
    if (ble_on) {
        g_ble_events = 0;
    }
    SYST_CSR = SYST_CSR_ENABLE | SYST_CSR_TICKINT | SYST_CSR_CLKSOURCE;

    start = board_time_us();
    for (;;) {
        uint32_t due;

        now = board_time_us();
        if (now - start >= RUN_DURATION_US) {
            break;
        }

        /* Inside a BLE connection event the link-layer stack owns the CPU: the
         * drain loop is unavailable. Model that as the CPU parking (WFI) WITHOUT
         * draining until the event window closes. The PPG FIFO backs up during
         * the window and overflows — the bug under test — and because the park
         * is idle, the simulator fast-forwards it. */
        if (ble_on && now < g_busy_until) {
            cpu_sleep();
            continue;
        }

        /* ---- PPG: drain every pending sample this wake ---- */
        ppg_drained += board_ppg_drain();

        /* ---- producer: rate-controlled acquisition ---- */
        due = sm_rate_poll(&mgr.imu_rate, now);
        while (due--) {
            int16_t accel[3], gyro[3];

            if (board_imu_read(accel, gyro) == 0) {
                sm_submit_imu(&mgr, accel, gyro, now);
            }
        }

        due = sm_rate_poll(&mgr.temp_rate, now);
        while (due--) {
            sm_submit_temp(&mgr, board_temp_read_milli_c(), now);
        }

        /* ---- consumer: what the GATT notify path would send ---- */
        if (sm_pending_imu(&mgr) > 0) {
            sm_imu_sample_t batch[64];
            uint8_t         frame[BLE_DEFAULT_PAYLOAD];
            sm_stats_t      st;
            size_t          n, consumed, len;

            sm_get_stats(&mgr, &st);
            n = sm_peek_imu(&mgr, batch, 64);
            if (n > 0) {
                len = ble_batch_encode_imu(batch, n, st.imu.dropped, frame,
                                           sizeof(frame), &consumed);
                if (len > 0) {
                    sm_release_imu(&mgr, consumed);
                    frames++;
                }
            }
        }

        /* Serviced this tick: sleep until the next SysTick. This is the idle
         * the simulator fast-forwards; a busy re-poll here would keep the CPU
         * hot and defeat the whole point. */
        cpu_sleep();
    }

    /* Stop the radio model before leaving the pass. */
    g_ble_on = 0;
    SYST_CSR = 0;

    {
        sm_stats_t st;
        sm_get_stats(&mgr, &st);
        res->imu_pushed = st.imu.pushed;
    }
    res->ppg_drained = ppg_drained;
    res->frames      = frames;
    res->ovf         = board_ppg_ovf();
    res->events      = g_ble_events;
}

int main(void)
{
    int imu_ok, temp_ok;
    pass_result_t off = {0}, on = {0};

    board_console_init();
    board_time_init();
    board_i2c_init();
    board_motor_gpio_init();

    board_puts("\r\n=== smart ring / nRF54L15 ===\r\n");
    board_puts("BOM: imu BMI270 / ppg MAX30102 / temp TMP117 / touch GPIO / haptic DRV2605\r\n");

    /* ---- bring-up ---- */
    imu_ok  = (board_imu_init() == 0);
    board_puts(imu_ok
                   ? "[init] imu    BMI270   ok (CHIP_ID=0x24, init_ok)\r\n"
                   : "[init] imu    BMI270   FAIL\r\n");
    board_puts(board_ppg_init() == 0
                   ? "[init] ppg    MAX30102 ok (PART_ID=0x15, 1600 Hz)\r\n"
                   : "[init] ppg    MAX30102 FAIL\r\n");
    temp_ok = (board_skin_temp_init() == 0);
    board_puts(temp_ok
                   ? "[init] temp   TMP117   ok (DEVICE_ID=0x0117)\r\n"
                   : "[init] temp   TMP117   FAIL\r\n");
    board_puts(board_haptic_init() == 0
                   ? "[init] haptic DRV2605  ok\r\n"
                   : "[init] haptic DRV2605  FAIL\r\n");

    /* ---- one-shot sensor read-out (proves every peripheral responds) ---- */
    {
        uint8_t touch = 0;

        print_kv("steps (BMI270): ", (int32_t)board_steps());
        print_kv("skin temp mC  : ", board_temp_read_milli_c());
        (void)board_touch_read(&touch);
        print_kv("touch bit P13 : ", (int32_t)touch);
        board_motor_enable(1);
        board_puts(board_haptic_buzz() == 0
                       ? "haptic        : GO asserted\r\n"
                       : "haptic        : buzz FAIL\r\n");
        board_motor_enable(0);
    }

    /*
     * ---- phase 0: software ring-buffer overflow + recovery ----
     * "Phone out of range": the BLE consumer stalls for ~2.4 s while the 50 Hz
     * IMU keeps producing. The 100-deep software ring overflows gracefully
     * (overwrite-oldest), and on resume the sequence gap is reported exactly —
     * proving loss is COUNTED, not silent. This is the Part-2 requirement,
     * distinct from the Part-3 PPG-hardware-FIFO/BLE bug demonstrated below.
     */
    {
        stall_result_t sr = {0};

        board_puts("\r\n--- phase 0: ring-buffer overflow + recovery (phone out of range) ---\r\n");
        run_stall_recovery(&sr);
        print_kv("imu  pushed        : ", (int32_t)sr.pushed);
        print_kv("imu  delivered     : ", (int32_t)sr.delivered);
        print_kv("imu  dropped (overwrite-oldest): ", (int32_t)sr.dropped);
        board_puts("buffered           : ");
        board_put_i32((int32_t)sr.buffered);
        board_puts("/");
        board_put_i32((int32_t)sr.capacity);
        board_puts(" (newest retained)\r\n");
        print_kv("sequence gap: lost : ", (int32_t)sr.seq_gap);
        board_puts((sr.dropped > 0 && sr.buffered == sr.capacity &&
                    sr.seq_gap == sr.dropped)
                       ? "overflow handled gracefully: loss counted, latest 100 retained.\r\n"
                       : "inconclusive: overflow accounting did not close.\r\n");
    }

    /*
     * The headline metric is OVERFLOW = samples the FIFO lost. For BLE-off the
     * drain count is also meaningful: it equals the samples produced (rate ×
     * duration ≈ 960), proving the driver kept up. For BLE-on it is NOT reported
     * as "drained": once a service falls behind and the FIFO overflows, the
     * polled one-sample-per-transaction drain thrashes (each read itself burns
     * GRTC time, during which the 1600 Hz source keeps producing), so the raw
     * count reflects wasted re-reads, not clean samples. Overflow is the honest
     * signal; a real driver would burst-read on the A_FULL interrupt instead.
     */
    board_puts("\r\n--- pass 1: BLE off (CPU free to drain) ---\r\n");
    run_pass(0, &off);
    print_kv("imu  pushed        : ", (int32_t)off.imu_pushed);
    print_kv("ppg  drained (==produced): ", (int32_t)off.ppg_drained);
    print_kv("ble  frames        : ", (int32_t)off.frames);
    print_kv("ppg  lost to overflow: ", (int32_t)off.ovf);

    /* ---- pass 2: BLE on ---- */
    board_puts("\r\n--- pass 2: BLE on (SysTick steals the CPU) ---\r\n");
    run_pass(1, &on);
    print_kv("imu  pushed        : ", (int32_t)on.imu_pushed);
    print_kv("ble  frames        : ", (int32_t)on.frames);
    print_kv("ble  events        : ", (int32_t)on.events);
    print_kv("ppg  lost to overflow (sat@31): ", (int32_t)on.ovf);

    /* ---- verdict ---- */
    board_puts("\r\n=== result ===\r\n");
    board_puts((off.ovf == 0 && on.ovf > 0)
                   ? "PPG FIFO overflows ONLY under BLE load — bug reproduced.\r\n"
                   : "inconclusive: overflow pattern did not separate the passes.\r\n");
    board_puts("(digital/timing model: CPU starvation + poll jitter, not RF supply noise)\r\n");
    board_puts("=== done ===\r\n");

    (void)imu_ok;
    (void)temp_ok;
    for (;;) {
    }
}
