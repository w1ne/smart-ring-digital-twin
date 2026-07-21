/*
 * Board support interface for the nRF54L15 smart ring.
 *
 * This is the HAL/driver layer from the architecture document: the sensor
 * manager core sits above it and never includes this header.
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

void board_console_init(void);
void board_puts(const char *s);
void board_put_i32(int32_t v);

void     board_time_init(void);
uint64_t board_time_us(void);

void board_i2c_init(void);
int  board_i2c_write(uint8_t addr, const uint8_t *data, uint32_t len);
int  board_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *dst, uint32_t len);
int  board_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val);

/* Each init returns 0 only after reading back the device's ID register, so a
 * silent bus failure cannot be mistaken for a working sensor. */
int      board_imu_init(void);           /* BMI270: reset + config-load + probe */
int      board_imu_read(int16_t accel[3], int16_t gyro[3]);
uint32_t board_steps(void);              /* BMI270 hardware step counter */

int  board_ppg_init(void);
int  board_ppg_read(uint32_t *red, uint32_t *ir);
/* Drain every pending FIFO sample; returns the count drained. Keeping the FIFO
 * empty is exactly what "the CPU kept up" means. */
uint32_t board_ppg_drain(void);
uint8_t  board_ppg_ovf(void);            /* OVF_COUNTER (0x05): lost samples */
void     board_ppg_reset_fifo(void);     /* zero WR/OVF/RD pointers */

int      board_skin_temp_init(void);     /* TMP117: DEVICE_ID == 0x0117 */
int32_t  board_temp_read_milli_c(void);  /* TMP117 TEMP_RESULT, milli-°C */

int  board_haptic_init(void);
int  board_haptic_buzz(void);
int  board_touch_read(uint8_t *pressed); /* 1-ch GPIO cap sensor, P1.13 */

void board_motor_gpio_init(void);
void board_motor_enable(int on);

#endif /* BOARD_H */
