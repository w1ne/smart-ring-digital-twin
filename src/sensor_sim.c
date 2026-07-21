#include "sensor_sim.h"

/* Length of the sine lookup table. A power of two so the phase index can wrap
 * with a mask (SIN_TABLE_LEN - 1) rather than a modulo. */
#define SIN_TABLE_LEN 64

/* Accelerometer scale: at +/-8g full scale over 16-bit signed, 1g = 4096 LSB.
 * The synthetic Z axis rests here to carry gravity. */
#define ACCEL_LSB_PER_G 4096

/* Integer sine approximation over SIN_TABLE_LEN steps, scaled to +/-1000.
 * A table keeps the simulator free of libm so it can be dropped into a
 * firmware build without pulling in floating point. */
static const int16_t k_sin64[SIN_TABLE_LEN] = {
    0,    98,   195,  290,  383,  471,  556,  634,  707,  773,   831,  881,  924,
    957,  981,  995,  1000, 995,  981,  957,  924,  881,  831,   773,  707,  634,
    556,  471,  383,  290,  195,  98,   0,    -98,  -195, -290,  -383, -471, -556,
    -634, -707, -773, -831, -881, -924, -957, -981, -995, -1000, -995, -981, -957,
    -924, -881, -831, -773, -707, -634, -556, -471, -383, -290,  -195, -98,
};

static uint32_t xorshift32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

/* Zero-mean noise in [-amp, amp]. */
static int16_t noise(sensor_sim_t *sim, int16_t amp)
{
    return (int16_t)((int32_t)(xorshift32(&sim->state) % (uint32_t)(2 * amp + 1)) - amp);
}

void sensor_sim_init(sensor_sim_t *sim, uint32_t seed)
{
    sim->state = seed ? seed : 1u; /* xorshift32 is degenerate at zero */
    sim->n     = 0;
}

void sensor_sim_imu(sensor_sim_t *sim, int16_t accel[3], int16_t gyro[3])
{
    /* ~1.3 Hz arm swing at 50 Hz sampling: 64-step period over ~38 samples.
     * The +16 (quarter of SIN_TABLE_LEN) gives a 90-degree phase for quad. */
    int16_t phase = k_sin64[(sim->n * 12 / 7) & (SIN_TABLE_LEN - 1)];
    int16_t quad  = k_sin64[((sim->n * 12 / 7) + 16) & (SIN_TABLE_LEN - 1)];

    /* +/-8g full scale, 16-bit signed => 4096 LSB/g. Z carries gravity. */
    accel[0] = (int16_t)(phase / 2 + noise(sim, 40));
    accel[1] = (int16_t)(quad / 4 + noise(sim, 40));
    accel[2] = (int16_t)(ACCEL_LSB_PER_G + phase / 8 + noise(sim, 40));

    /* +/-2000 dps full scale, 16-bit signed => ~16.4 LSB/dps. */
    gyro[0] = (int16_t)(quad * 2 + noise(sim, 25));
    gyro[1] = (int16_t)(phase + noise(sim, 25));
    gyro[2] = (int16_t)(noise(sim, 60));

    sim->n++;
}

int32_t sensor_sim_temp(sensor_sim_t *sim)
{
    /* Skin temperature: slow drift around 33.000 C plus +/-30 mK of noise,
     * which is roughly what a decent digital sensor gives you. */
    int32_t drift = k_sin64[(sim->n / 8) & (SIN_TABLE_LEN - 1)] / 4; /* +/-250 mK, slow */
    return 33000 + drift + noise(sim, 30);
}
