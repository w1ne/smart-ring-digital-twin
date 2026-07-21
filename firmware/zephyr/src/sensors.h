/*
 * sensors.h - thin driver over the four smart-ring I2C sensors, using the raw
 * Zephyr I2C API (no per-sensor driver bindings). The app only needs to probe
 * IDs once and then read the IMU and skin-temperature data streams.
 */
#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Probe WHO_AM_I / ID on all four sensors and power up the IMU. Prints one
 * line per sensor. Returns true only if the two data-producing sensors
 * (BMI270 IMU + TMP117 temperature) both answered with the expected ID; the
 * PPG and haptic are probed for completeness but are not part of the pipeline.
 */
bool sensors_init(void);

/* Read the BMI270's 6-axis data (raw sensor LSB). Returns 0 on success. */
int sensors_read_imu(int16_t accel[3], int16_t gyro[3]);

/* Read the TMP117 skin temperature in milli-degrees Celsius. 0 on success. */
int sensors_read_temp(int32_t *milli_celsius);

#endif /* SENSORS_H */
