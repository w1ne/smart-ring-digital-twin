/*
 * sensor_sim.h - deterministic simulated sensor sources.
 *
 * Stands in for the real IMU and temperature drivers. Deterministic by
 * construction (seeded xorshift, no wall-clock input) so a failing test run
 * reproduces exactly. Signals are shaped to be plausible rather than
 * realistic: enough structure that a plot looks like a wrist, not enough to
 * pretend this is a validated sensor model.
 */
#ifndef SENSOR_SIM_H
#define SENSOR_SIM_H

#include <stdint.h>

typedef struct {
    uint32_t state; /* xorshift32 PRNG state */
    uint32_t n;     /* sample index, drives the synthetic gait phase */
} sensor_sim_t;

void sensor_sim_init(sensor_sim_t *sim, uint32_t seed);

/* Fill accel/gyro with the next simulated IMU sample (raw LSB, +/-8g and
 * +/-2000 dps full scale assumed). */
void sensor_sim_imu(sensor_sim_t *sim, int16_t accel[3], int16_t gyro[3]);

/* Next simulated skin temperature in milli-degrees Celsius, drifting slowly
 * around 33 C with sensor noise. */
int32_t sensor_sim_temp(sensor_sim_t *sim);

#endif /* SENSOR_SIM_H */
