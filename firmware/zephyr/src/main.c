/*
 * main.c - smart-ring app entry point. Initialises the sensors and the
 * portable sensor-manager core, then runs the acquisition loop: a 20 ms timer
 * paces reads of the BMI270 (50 Hz) and TMP117 (1 Hz) into the manager, and
 * once a second the manager's pushed/dropped/buffered stats are printed.
 */
#include <zephyr/kernel.h>
#include <zephyr/version.h>

#include "sensors.h"
#include "sensor_manager.h"
#include "ble.h"

/* Acquisition tick. The IMU runs at 50 Hz, so one tick == one IMU period; the
 * manager's rate controllers derive the 1 Hz temperature cadence from it. */
#define TICK_PERIOD  K_MSEC(20)
#define STATS_PERIOD_US 1000000ULL

/* The timer only marks time; the blocking I2C reads happen in the loop below
 * (thread context), woken by this semaphore. */
static K_SEM_DEFINE(tick_sem, 0, 1);

static void tick_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&tick_sem);
}

static K_TIMER_DEFINE(tick_timer, tick_expiry, NULL);

static void print_stats(sensor_manager_t *mgr)
{
	sm_stats_t s;

	sm_get_stats(mgr, &s);
	printk("stats: imu pushed=%u dropped=%u buffered=%u/%u missed=%u | "
	       "temp pushed=%u dropped=%u buffered=%u/%u missed=%u\n",
	       s.imu.pushed, s.imu.dropped, s.imu.count, s.imu.capacity,
	       s.imu_missed_deadlines, s.temp.pushed, s.temp.dropped, s.temp.count,
	       s.temp.capacity, s.temp_missed_deadlines);
}

int main(void)
{
	printk("\n=== smart-ring nRF54L15 (Zephyr %s) ===\n", KERNEL_VERSION_STRING);

	if (!sensors_init()) {
		printk("sensor bring-up failed; halting\n");
		return 0;
	}

	sensor_manager_t mgr;

	if (sm_init(&mgr, sm_time_now_us()) != SM_OK) {
		printk("sensor-manager init failed; halting\n");
		return 0;
	}

	ble_start(); /* advertises the Ring Data Service on the DK; no-op without CONFIG_BT */

	k_timer_start(&tick_timer, TICK_PERIOD, TICK_PERIOD);

	uint64_t last_stats_us = sm_time_now_us();

	for (;;) {
		k_sem_take(&tick_sem, K_FOREVER);

		uint64_t now_us = sm_time_now_us();

		if (sm_rate_poll(&mgr.imu_rate, now_us)) {
			int16_t accel[3], gyro[3];

			if (sensors_read_imu(accel, gyro) == 0) {
				sm_submit_imu(&mgr, accel, gyro, now_us);
			}
		}

		if (sm_rate_poll(&mgr.temp_rate, now_us)) {
			int32_t milli_celsius;

			if (sensors_read_temp(&milli_celsius) == 0) {
				sm_submit_temp(&mgr, milli_celsius, now_us);
			}
		}

		if (now_us - last_stats_us >= STATS_PERIOD_US) {
			print_stats(&mgr);
			ble_notify_imu(&mgr); /* drain a batch to the phone; no-op without CONFIG_BT */
			last_stats_us = now_us;
		}
	}

	return 0;
}
