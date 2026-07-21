/*
 * sensors.c - raw-I2C access to the smart-ring sensor bus (TWIM21 / i2c21).
 */
#include "sensors.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

/* 7-bit bus addresses (see the board overlay for the bus topology). */
#define ADDR_IMU    0x68 /* BMI270  */
#define ADDR_PPG    0x57 /* MAX30102 */
#define ADDR_TEMP   0x48 /* TMP117  */
#define ADDR_HAPTIC 0x5A /* DRV2605 */

/* BMI270: CHIP_ID register and its fixed value; 6-axis data burst. */
#define BMI270_REG_CHIP_ID 0x00
#define BMI270_CHIP_ID     0x24
#define BMI270_REG_PWR_CTRL 0x7D
#define BMI270_PWR_ACC_GYR  0x0E /* enable accel + gyro (+ temp) */
#define BMI270_REG_ACC_DATA 0x0C /* accel[3] then gyro[3], 12 LE bytes */
#define BMI270_DATA_LEN     12

/* TMP117: DEVICE_ID (0x0117) and the temperature result register. */
#define TMP117_REG_TEMP      0x00
#define TMP117_REG_DEVICE_ID 0x0F
#define TMP117_DEVICE_ID     0x0117
/* 7.8125 m°C per LSB == 125/16 mC. */
#define TMP117_MC_NUM 125
#define TMP117_MC_DEN 16

/* MAX30102: PART_ID register and value (PPG, probed only). */
#define MAX30102_REG_PART_ID 0xFF
#define MAX30102_PART_ID     0x15

/* DRV2605: STATUS register; the top 3 bits carry the device ID (7). */
#define DRV2605_REG_STATUS    0x00
#define DRV2605_DEVICE_ID     7
#define DRV2605_ID_SHIFT      5

static const struct device *const i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c21));

/* Read a big-endian 16-bit register (TMP117 layout: MSB first). */
static int read_reg16_be(uint8_t addr, uint8_t reg, uint16_t *out)
{
	uint8_t buf[2];
	int rc = i2c_burst_read(i2c_bus, addr, reg, buf, sizeof(buf));

	if (rc == 0) {
		*out = ((uint16_t)buf[0] << 8) | buf[1];
	}
	return rc;
}

bool sensors_init(void)
{
	if (!device_is_ready(i2c_bus)) {
		printk("i2c bus not ready\n");
		return false;
	}

	uint8_t chip_id = 0, part_id = 0, status = 0;
	uint16_t device_id = 0;
	bool imu_ok, temp_ok, ppg_ok, haptic_ok;

	imu_ok = i2c_reg_read_byte(i2c_bus, ADDR_IMU, BMI270_REG_CHIP_ID, &chip_id) == 0 &&
		 chip_id == BMI270_CHIP_ID;
	temp_ok = read_reg16_be(ADDR_TEMP, TMP117_REG_DEVICE_ID, &device_id) == 0 &&
		  device_id == TMP117_DEVICE_ID;
	ppg_ok = i2c_reg_read_byte(i2c_bus, ADDR_PPG, MAX30102_REG_PART_ID, &part_id) == 0 &&
		 part_id == MAX30102_PART_ID;
	haptic_ok = i2c_reg_read_byte(i2c_bus, ADDR_HAPTIC, DRV2605_REG_STATUS, &status) == 0 &&
		    (status >> DRV2605_ID_SHIFT) == DRV2605_DEVICE_ID;

	printk("probe: imu(bmi270) id=0x%02x %s\n", chip_id, imu_ok ? "OK" : "FAIL");
	printk("probe: temp(tmp117) id=0x%04x %s\n", device_id, temp_ok ? "OK" : "FAIL");
	printk("probe: ppg(max30102) id=0x%02x %s\n", part_id, ppg_ok ? "OK" : "FAIL");
	printk("probe: haptic(drv2605) id=%u %s\n", status >> DRV2605_ID_SHIFT,
	       haptic_ok ? "OK" : "FAIL");

	/*
	 * Power up the IMU's accel + gyro. A real BMI270 also needs its config
	 * blob uploaded here; that DK-only step is omitted deliberately (the
	 * data registers are all the pipeline reads).
	 */
	if (imu_ok) {
		i2c_reg_write_byte(i2c_bus, ADDR_IMU, BMI270_REG_PWR_CTRL, BMI270_PWR_ACC_GYR);
	}

	return imu_ok && temp_ok;
}

int sensors_read_imu(int16_t accel[3], int16_t gyro[3])
{
	uint8_t d[BMI270_DATA_LEN];
	int rc = i2c_burst_read(i2c_bus, ADDR_IMU, BMI270_REG_ACC_DATA, d, sizeof(d));

	if (rc != 0) {
		return rc;
	}
	for (int i = 0; i < 3; i++) {
		accel[i] = (int16_t)(d[2 * i] | (d[2 * i + 1] << 8));
		gyro[i] = (int16_t)(d[6 + 2 * i] | (d[6 + 2 * i + 1] << 8));
	}
	return 0;
}

int sensors_read_temp(int32_t *milli_celsius)
{
	uint16_t raw;
	int rc = read_reg16_be(ADDR_TEMP, TMP117_REG_TEMP, &raw);

	if (rc == 0) {
		*milli_celsius = (int32_t)(int16_t)raw * TMP117_MC_NUM / TMP117_MC_DEN;
	}
	return rc;
}
