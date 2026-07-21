/*
 * demo_main.c - runs the sensor manager against simulated sensors on a host.
 *
 * Topology mirrors the target:
 *
 *   acquisition thread  - drift-free rate control, 50 Hz IMU / 1 Hz temp,
 *                         stands in for the DRDY interrupt and the 1 Hz timer
 *   ble thread          - wakes once per simulated connection interval,
 *                         drains in batches, encodes GATT notification frames
 *   main                - prints periodic stats
 *
 * Two modes, selected on the command line:
 *
 *   (default)   connected: the BLE consumer drains every connection interval
 *   --stall N   the phone goes out of range for N seconds mid-run
 *
 * The stall mode is the interesting one: it demonstrates graceful overflow.
 * The device keeps sampling, the buffer keeps the newest 100 IMU samples, the
 * drop counter records exactly what was lost, and when the link comes back
 * the first frame carries a sequence gap the phone can act on.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>

#include "ble_batch.h"
#include "sensor_manager.h"
#include "sensor_sim.h"
#include "sm_port.h"

/* Simulated BLE connection interval. 50 ms is a plausible "research mode"
 * value; see the power tables in docs/part1-architecture.md for why a
 * week-mode device would use 400-1000 ms instead. */
#define CONN_INTERVAL_MS 50

/* Default run length when --seconds is not given, in seconds. */
#define DEMO_DEFAULT_RUN_S 10

/* Per-drain staging capacities for the BLE consumer. Each is comfortably
 * larger than one payload's worth of records (16 IMU / ~38 temp), so a single
 * peek always offers the encoder more than it can pack in one frame. */
#define DEMO_IMU_DRAIN  64
#define DEMO_TEMP_DRAIN 32

static sensor_manager_t mgr;
static volatile bool    g_running = true;
static volatile bool    g_link_up = true;

static struct {
    unsigned frames;
    unsigned samples;
    unsigned bytes;
    unsigned gaps;
} g_ble;

static void sleep_ms(unsigned ms)
{
    struct timespec ts = {.tv_sec = ms / 1000, .tv_nsec = (long)(ms % 1000) * 1000000L};
    nanosleep(&ts, NULL);
}

static void *acquisition_thread(void *arg)
{
    sensor_sim_t sim;

    (void)arg;
    sensor_sim_init(&sim, 0xA5A5u);

    while (g_running) {
        uint64_t now = sm_time_now_us();
        uint32_t due;

        due = sm_rate_poll(&mgr.imu_rate, now);
        while (due--) {
            int16_t a[3], g[3];
            sensor_sim_imu(&sim, a, g);
            sm_submit_imu(&mgr, a, g, now);
        }

        due = sm_rate_poll(&mgr.temp_rate, now);
        while (due--) {
            sm_submit_temp(&mgr, sensor_sim_temp(&sim), now);
        }

        /* Poll at 4x the fastest sensor rate. On the target this thread does
         * not exist: the IMU DRDY interrupt and a 1 Hz RTC alarm drive
         * acquisition and the CPU sleeps in between. */
        sleep_ms(5);
    }
    return NULL;
}

static void *ble_thread(void *arg)
{
    uint32_t last_seq  = 0;
    bool     have_last = false;

    (void)arg;

    while (g_running) {
        sm_imu_sample_t  imu[DEMO_IMU_DRAIN];
        sm_temp_sample_t tmp[DEMO_TEMP_DRAIN];
        uint8_t          frame[BLE_DEFAULT_PAYLOAD];
        sm_stats_t       st;
        size_t           n, consumed, len;

        sleep_ms(CONN_INTERVAL_MS);

        if (!g_link_up) {
            continue; /* out of range: no connection events at all */
        }

        sm_get_stats(&mgr, &st);

        /* IMU: peek, encode into one notification, release only what fit. */
        n = sm_peek_imu(&mgr, imu, DEMO_IMU_DRAIN);
        if (n > 0) {
            len = ble_batch_encode_imu(imu, n, st.imu.dropped, frame, sizeof(frame),
                                       &consumed);
            if (len > 0) {
                sm_release_imu(&mgr, consumed);

                if (have_last && ble_batch_first_seq(frame) != last_seq + 1) {
                    g_ble.gaps++;
                    printf("  [ble] sequence gap: expected %u, got %u (%u lost)\n",
                           last_seq + 1, ble_batch_first_seq(frame),
                           ble_batch_first_seq(frame) - last_seq - 1);
                }
                last_seq  = ble_batch_first_seq(frame) + ble_batch_count(frame) - 1;
                have_last = true;

                g_ble.frames++;
                g_ble.samples += ble_batch_count(frame);
                g_ble.bytes += (unsigned)len;
            }
        }

        /* Temperature: same pattern, its own frame type. */
        n = sm_peek_temp(&mgr, tmp, DEMO_TEMP_DRAIN);
        if (n > 0) {
            len = ble_batch_encode_temp(tmp, n, st.temp.dropped, frame, sizeof(frame),
                                        &consumed);
            if (len > 0) {
                sm_release_temp(&mgr, consumed);
                g_ble.frames++;
                g_ble.bytes += (unsigned)len;
            }
        }
    }
    return NULL;
}

static void print_stats(const char *label)
{
    sm_stats_t st;

    sm_get_stats(&mgr, &st);
    printf("[%s]\n", label);
    printf("  imu   pushed=%-6u dropped=%-6u buffered=%u/%u\n", st.imu.pushed,
           st.imu.dropped, st.imu.count, st.imu.capacity);
    printf("  temp  pushed=%-6u dropped=%-6u buffered=%u/%u\n", st.temp.pushed,
           st.temp.dropped, st.temp.count, st.temp.capacity);
    printf("  missed deadlines: imu=%u temp=%u\n", st.imu_missed_deadlines,
           st.temp_missed_deadlines);
    printf("  ble   frames=%u samples=%u bytes=%u gaps=%u\n", g_ble.frames, g_ble.samples,
           g_ble.bytes, g_ble.gaps);
}

int main(int argc, char **argv)
{
    pthread_t t_acq, t_ble;
    unsigned  run_s   = DEMO_DEFAULT_RUN_S;
    unsigned  stall_s = 0;
    int       i;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--stall") == 0 && i + 1 < argc) {
            stall_s = (unsigned)strtoul(argv[++i], NULL, 10);
        } else if (strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
            run_s = (unsigned)strtoul(argv[++i], NULL, 10);
        } else {
            fprintf(stderr, "usage: %s [--seconds N] [--stall N]\n", argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (sm_init(&mgr, sm_time_now_us()) != SM_OK) {
        fprintf(stderr, "sm_init failed\n");
        return EXIT_FAILURE;
    }

    printf("smart-ring sensor manager demo\n");
    printf("  imu %u Hz (keep latest %u), temp %u Hz (keep latest %u)\n", SM_IMU_RATE_HZ,
           SM_IMU_CAPACITY, SM_TEMP_RATE_HZ, SM_TEMP_CAPACITY);
    printf("  simulated connection interval %u ms, ATT payload %u B\n", CONN_INTERVAL_MS,
           BLE_DEFAULT_PAYLOAD);
    printf("  running %u s%s\n\n", run_s, stall_s ? " with a mid-run link stall" : "");

    pthread_create(&t_acq, NULL, acquisition_thread, NULL);
    pthread_create(&t_ble, NULL, ble_thread, NULL);

    if (stall_s > 0 && stall_s < run_s) {
        /* Run connected for the first third, then stall, then recover. */
        unsigned before = run_s / 3;

        sleep_ms(before * 1000);
        print_stats("connected");

        printf("\n  --- phone out of range for %u s ---\n\n", stall_s);
        g_link_up = false;
        sleep_ms(stall_s * 1000);
        g_link_up = true;
        print_stats("after stall");

        printf("\n  --- link restored ---\n\n");
        sleep_ms((run_s - before - stall_s) * 1000);
    } else {
        sleep_ms(run_s * 1000);
    }

    g_running = false;
    pthread_join(t_acq, NULL);
    pthread_join(t_ble, NULL);

    printf("\n");
    print_stats("final");

    {
        sm_stats_t st;
        sm_get_stats(&mgr, &st);
        printf("\n  delivered %u of %u imu samples (%u dropped, %u still buffered)\n",
               g_ble.samples, st.imu.pushed, st.imu.dropped, st.imu.count);
        printf("  %u notifications for %u samples: %.1f samples per radio event\n",
               g_ble.frames, g_ble.samples,
               g_ble.frames ? (double)g_ble.samples / g_ble.frames : 0.0);
    }

    sm_deinit(&mgr);
    return EXIT_SUCCESS;
}
