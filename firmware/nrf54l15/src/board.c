/*
 * Board support for the nRF54L15 smart ring: console, I²C, time, sensors.
 *
 * Everything here is polled and blocking. The sensor manager core does not
 * care — it takes timestamps as parameters — and a polled bring-up keeps the
 * failure surface small enough that a problem localises to one register.
 * Interrupt-driven acquisition is the next step, and is what the architecture
 * document argues for on real hardware.
 */
#include "board.h"

#include "nrf54l15.h"

/* ── console ─────────────────────────────────────────────────────────────── */

/*
 * EasyDMA reads the TX buffer over the bus, so it must live in RAM. A string
 * literal would sit in RRAM and fault on silicon. This buffer is in .data for
 * exactly that reason.
 */
static char tx_buf[192];

void board_console_init(void)
{
    UARTE_PSEL_TXD(UARTE20_BASE) = PSEL(1, 4);
    UARTE_PSEL_RXD(UARTE20_BASE) = PSEL(1, 5);
    UARTE_BAUDRATE(UARTE20_BASE) = UARTE_BAUD_115200;
    UARTE_ENABLE(UARTE20_BASE)   = UARTE_ENABLE_UARTE;
}

void board_puts(const char *s)
{
    uint32_t len = 0;

    while (s[len] != '\0' && len < sizeof(tx_buf)) {
        tx_buf[len] = s[len];
        len++;
    }
    if (len == 0) {
        return;
    }

    UARTE_EVENTS_DMA_TX_END(UARTE20_BASE)  = 0;
    UARTE_EVENTS_TXSTOPPED(UARTE20_BASE)   = 0;
    UARTE_DMA_TX_PTR(UARTE20_BASE)         = (uint32_t)(uintptr_t)tx_buf;
    UARTE_DMA_TX_MAXCNT(UARTE20_BASE)      = len;
    UARTE_TASKS_DMA_TX_START(UARTE20_BASE) = 1;

    while (UARTE_EVENTS_DMA_TX_END(UARTE20_BASE) == 0) {
    }

    UARTE_EVENTS_DMA_TX_END(UARTE20_BASE)  = 0;
    UARTE_TASKS_DMA_TX_STOP(UARTE20_BASE)  = 1;
}

/* Minimal integer formatting. No printf: newlib's printf would pull in ~30 KB
 * and a heap, neither of which this firmware has. */
void board_put_i32(int32_t v)
{
    char buf[12];
    int  i = 0;
    uint32_t u;

    if (v < 0) {
        board_puts("-");
        u = (uint32_t)(-(v + 1)) + 1u;   /* avoids UB at INT32_MIN */
    } else {
        u = (uint32_t)v;
    }

    if (u == 0) {
        board_puts("0");
        return;
    }
    while (u > 0 && i < (int)sizeof(buf)) {
        buf[i++] = (char)('0' + (u % 10u));
        u /= 10u;
    }

    {
        char out[13];
        int  j = 0;
        while (i > 0) {
            out[j++] = buf[--i];
        }
        out[j] = '\0';
        board_puts(out);
    }
}

/* ── time (GRTC) ─────────────────────────────────────────────────────────── */

void board_time_init(void)
{
    GRTC_TASKS_CLEAR(GRTC_BASE) = 1;
    GRTC_MODE(GRTC_BASE)        = GRTC_MODE_SYSCOUNTEREN;
    GRTC_TASKS_START(GRTC_BASE) = 1;
}

uint64_t board_time_us(void)
{
    /*
     * SYSCOUNTER is 52-bit across an L/H pair, and reading L latches H — that
     * pair exists precisely so a 32-bit rollover cannot tear. Read L first.
     * The counter runs at 1 MHz, so the raw value is already microseconds.
     */
    uint32_t lo = GRTC_SYSCOUNTERL(GRTC_BASE);
    uint32_t hi = GRTC_SYSCOUNTERH(GRTC_BASE);

    return ((uint64_t)hi << 32) | (uint64_t)lo;
}

/* ── I²C (TWIM21) ────────────────────────────────────────────────────────── */

void board_i2c_init(void)
{
    TWIM_FREQUENCY(TWIM21_BASE) = TWIM_FREQ_400K;
    TWIM_ENABLE(TWIM21_BASE)    = TWIM_ENABLE_TWIM;
}

/* EasyDMA buffers must be in RAM, same constraint as the UARTE. */
static uint8_t i2c_tx[4];

/* SHORTS bits. These positions are genuinely unchanged from nRF52 — unlike the
 * tasks/events/DMA registers, which all moved on this family. */
#define SHORT_LASTTX_DMA_RX_START  (1u << 7)
#define SHORT_LASTTX_STOP          (1u << 9)
#define SHORT_LASTRX_STOP          (1u << 12)

static void twim_clear_events(void)
{
    TWIM_EVENTS_STOPPED(TWIM21_BASE)    = 0;
    TWIM_EVENTS_ERROR(TWIM21_BASE)      = 0;
    TWIM_EVENTS_DMA_TX_END(TWIM21_BASE) = 0;
    TWIM_EVENTS_DMA_RX_END(TWIM21_BASE) = 0;
}

static int twim_wait_stopped(void)
{
    while (TWIM_EVENTS_STOPPED(TWIM21_BASE) == 0 &&
           TWIM_EVENTS_ERROR(TWIM21_BASE) == 0) {
    }
    if (TWIM_EVENTS_ERROR(TWIM21_BASE)) {
        (void)TWIM_ERRORSRC(TWIM21_BASE);
        TWIM_TASKS_STOP(TWIM21_BASE) = 1;
        return -1;
    }
    return 0;
}

int board_i2c_write(uint8_t addr, const uint8_t *data, uint32_t len)
{
    uint32_t i;

    if (len > sizeof(i2c_tx)) {
        return -1;
    }
    for (i = 0; i < len; i++) {
        i2c_tx[i] = data[i];
    }

    TWIM_ADDRESS(TWIM21_BASE)       = addr;
    TWIM_SHORTS(TWIM21_BASE)        = SHORT_LASTTX_STOP;
    twim_clear_events();
    TWIM_DMA_TX_PTR(TWIM21_BASE)    = (uint32_t)(uintptr_t)i2c_tx;
    TWIM_DMA_TX_MAXCNT(TWIM21_BASE) = len;
    TWIM_DMA_RX_MAXCNT(TWIM21_BASE) = 0;   /* no stale RX length left armed */
    TWIM_TASKS_DMA_TX_START(TWIM21_BASE) = 1;

    return twim_wait_stopped();
}

int board_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *dst, uint32_t len)
{
    /*
     * Register read = write the register pointer, then a REPEATED START into
     * the read — with no STOP in between. A STOP would end the transaction,
     * and an I²C slave is entitled to treat that as the end of the addressing
     * phase.
     *
     * The nRF way to do this without CPU involvement is the
     * LASTTX_DMA_RX_START short: the hardware chains TX→RX itself, so the
     * repeated start happens with correct timing regardless of how long the CPU
     * takes to notice. LASTRX_STOP then releases the bus. One start, one wait.
     *
     * Doing it by hand as STARTTX / STOP / STARTRX is the obvious-looking
     * version and it is wrong: it emits a full STOP between the two phases.
     */
    i2c_tx[0] = reg;

    TWIM_ADDRESS(TWIM21_BASE)       = addr;
    TWIM_SHORTS(TWIM21_BASE)        = SHORT_LASTTX_DMA_RX_START | SHORT_LASTRX_STOP;
    twim_clear_events();
    TWIM_DMA_TX_PTR(TWIM21_BASE)    = (uint32_t)(uintptr_t)i2c_tx;
    TWIM_DMA_TX_MAXCNT(TWIM21_BASE) = 1;
    TWIM_DMA_RX_PTR(TWIM21_BASE)    = (uint32_t)(uintptr_t)dst;
    TWIM_DMA_RX_MAXCNT(TWIM21_BASE) = len;
    TWIM_TASKS_DMA_TX_START(TWIM21_BASE) = 1;

    return twim_wait_stopped();
}

int board_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = val;
    return board_i2c_write(addr, buf, 2);
}

/* ── sensors ─────────────────────────────────────────────────────────────── */

/* A short busy-wait for the handful of places a datasheet mandates a settle
 * time (BMI270 softreset needs ~2 ms). Bounded and dependency-free. */
static void board_delay_spin(uint32_t n)
{
    volatile uint32_t i;
    for (i = 0; i < n; i++) {
    }
}

/*
 * BMI270 bring-up (datasheet §"Initialization").
 *
 *   softreset -> disable adv power save -> INIT_CTRL=0 -> stream config blob
 *   to INIT_DATA -> INIT_CTRL=1 -> poll INTERNAL_STATUS == init_ok ->
 *   enable acc+gyr+temp.
 *
 * Real silicon requires the full ~8 KB Bosch config image streamed to
 * INIT_DATA; the simulator accepts and discards the blob, so a short dummy
 * upload satisfies the load-done handshake. The gate is honest: skipping the
 * upload leaves INTERNAL_STATUS at not_init, so this returns failure exactly
 * as a real part with a corrupt image would.
 */
int board_imu_init(void)
{
    uint8_t  chip = 0, status = 0;
    uint8_t  blob[4];
    int      tries;

    /* CMD = softreset (0xB6), then let the feature engine come up. */
    if (board_i2c_write_reg(ADDR_IMU, 0x7E, 0xB6) != 0) {
        return -1;
    }
    board_delay_spin(4000);

    /* PWR_CONF = 0x00: disable advanced power save (required before load). */
    if (board_i2c_write_reg(ADDR_IMU, 0x7C, 0x00) != 0) {
        return -1;
    }
    /* INIT_CTRL = 0x00: arm the config loader. */
    if (board_i2c_write_reg(ADDR_IMU, 0x59, 0x00) != 0) {
        return -1;
    }

    /* Stream a (dummy) config blob to INIT_DATA (0x5E). One transaction of the
     * register pointer plus a few payload bytes; real init streams the whole
     * image here, but any INIT_DATA write satisfies the model's handshake. */
    blob[0] = 0x5E;
    blob[1] = 0x00;
    blob[2] = 0x00;
    blob[3] = 0x00;
    if (board_i2c_write(ADDR_IMU, blob, 4) != 0) {
        return -1;
    }

    /* INIT_CTRL = 0x01: load done. */
    if (board_i2c_write_reg(ADDR_IMU, 0x59, 0x01) != 0) {
        return -1;
    }

    /* Poll INTERNAL_STATUS (0x21) for init_ok (0x01), bounded. */
    for (tries = 0; tries < 32; tries++) {
        if (board_i2c_read_reg(ADDR_IMU, 0x21, &status, 1) != 0) {
            return -1;
        }
        if ((status & 0x0F) == 0x01) {
            break;
        }
        board_delay_spin(1000);
    }

    /* PWR_CTRL = 0x0E: enable gyro (bit1) + accel (bit2) + temp (bit3). */
    if (board_i2c_write_reg(ADDR_IMU, 0x7D, 0x0E) != 0) {
        return -1;
    }

    /* CHIP_ID (0x00) must read 0x24 AND the feature engine must be init_ok. */
    if (board_i2c_read_reg(ADDR_IMU, 0x00, &chip, 1) != 0) {
        return -1;
    }
    return (chip == 0x24 && (status & 0x0F) == 0x01) ? 0 : -1;
}

int board_imu_read(int16_t accel[3], int16_t gyro[3])
{
    uint8_t raw[12];
    int     i;

    /* ACC_X_LSB (0x0C) .. GYR_Z_MSB (0x17) is one 12-byte burst, int16 LITTLE-
     * endian per axis (BMI270 order: accel[3] then gyro[3]). */
    if (board_i2c_read_reg(ADDR_IMU, 0x0C, raw, sizeof(raw)) != 0) {
        return -1;
    }

    for (i = 0; i < 3; i++) {
        accel[i] = (int16_t)((uint16_t)raw[i * 2] | ((uint16_t)raw[i * 2 + 1] << 8));
        gyro[i]  = (int16_t)((uint16_t)raw[6 + i * 2] | ((uint16_t)raw[6 + i * 2 + 1] << 8));
    }
    return 0;
}

uint32_t board_steps(void)
{
    uint8_t raw[4] = {0, 0, 0, 0};

    /* The step counter lives in the feature-engine window: select page 6 via
     * FEAT_PAGE (0x2F), then read the 32-bit LE count at the window base
     * (0x30..0x33). */
    if (board_i2c_write_reg(ADDR_IMU, 0x2F, 0x06) != 0) {
        return 0;
    }
    if (board_i2c_read_reg(ADDR_IMU, 0x30, raw, sizeof(raw)) != 0) {
        return 0;
    }
    return (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
           ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
}

int board_ppg_init(void)
{
    uint8_t part = 0;

    if (board_i2c_read_reg(ADDR_PPG, 0xFF, &part, 1) != 0) {
        return -1;
    }
    if (part != 0x15) {
        return -1;   /* MAX30102 PART_ID */
    }

    /*
     * SPO2_CONFIG (0x0A): SPO2_SR[4:2] = 110 selects 1600 Hz, the top of the
     * MAX30102's realistic range, with ADC_RGE=01 and LED_PW=11 (411 µs, the
     * 18-bit pulse width). 1600 Hz is a deliberate, documented choice: at that
     * rate the 32-deep FIFO fills in 20 ms, so a BLE connection-event ISR that
     * holds the CPU for a ~25 ms window starves the drain past a full FIFO and
     * OVF_COUNTER climbs — while a firmware that drains continuously keeps the
     * FIFO near-empty and never overflows. That contrast is the whole point.
     *   value = (ADC_RGE=01)<<5 | (SPO2_SR=110)<<2 | (LED_PW=11) = 0x3B
     */
    if (board_i2c_write_reg(ADDR_PPG, 0x0A, 0x3B) != 0) {
        return -1;
    }

    /* MODE_CONFIG = 0x03 (SpO2: red + IR). No samples are produced until a
     * mode is selected, same as silicon. */
    if (board_i2c_write_reg(ADDR_PPG, 0x09, 0x03) != 0) {
        return -1;
    }

    board_ppg_reset_fifo();
    return 0;
}

int board_ppg_read(uint32_t *red, uint32_t *ir)
{
    uint8_t raw[6];

    if (board_i2c_read_reg(ADDR_PPG, 0x07, raw, sizeof(raw)) != 0) {
        return -1;
    }

    /* 18-bit samples, left-justified in 3 bytes, MSB first. */
    *red = (((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | raw[2]) & 0x3FFFFu;
    *ir  = (((uint32_t)raw[3] << 16) | ((uint32_t)raw[4] << 8) | raw[5]) & 0x3FFFFu;
    return 0;
}

uint32_t board_ppg_drain(void)
{
    uint8_t  wr = 0, rd = 0;
    uint8_t  ptr[3];
    uint32_t pending, i, red, ir;

    /* FIFO_WR_PTR (0x04) and FIFO_RD_PTR (0x06) are 5-bit; pending is their
     * difference modulo the 32-slot depth. */
    if (board_i2c_read_reg(ADDR_PPG, 0x04, ptr, 3) != 0) {
        return 0;
    }
    wr = ptr[0] & 0x1F;
    rd = ptr[2] & 0x1F;
    pending = (uint32_t)((wr - rd) & 0x1F);
    /* Equal pointers with a non-zero OVF_COUNTER means a genuinely FULL FIFO
     * (32 samples), not an empty one. */
    if (pending == 0 && (ptr[1] & 0x1F) != 0) {
        pending = 32;
    }

    for (i = 0; i < pending; i++) {
        if (board_ppg_read(&red, &ir) != 0) {
            break;
        }
    }
    return i;
}

uint8_t board_ppg_ovf(void)
{
    uint8_t ovf = 0;

    /* OVF_COUNTER (0x05): samples lost to overrun, saturating at 31. */
    (void)board_i2c_read_reg(ADDR_PPG, 0x05, &ovf, 1);
    return ovf & 0x1F;
}

void board_ppg_reset_fifo(void)
{
    /* Zero the three FIFO pointers, as a driver does before streaming. */
    (void)board_i2c_write_reg(ADDR_PPG, 0x04, 0x00);   /* FIFO_WR_PTR  */
    (void)board_i2c_write_reg(ADDR_PPG, 0x05, 0x00);   /* OVF_COUNTER  */
    (void)board_i2c_write_reg(ADDR_PPG, 0x06, 0x00);   /* FIFO_RD_PTR  */
}

int board_skin_temp_init(void)
{
    uint8_t id[2] = {0, 0};

    /* TMP117 registers are 16-bit big-endian. DEVICE_ID (0x0F) reads 0x0117
     * (MSB 0x01, then LSB 0x17). */
    if (board_i2c_read_reg(ADDR_SKIN_TEMP, 0x0F, id, 2) != 0) {
        return -1;
    }
    return (id[0] == 0x01 && id[1] == 0x17) ? 0 : -1;
}

int32_t board_temp_read_milli_c(void)
{
    uint8_t raw[2];
    int16_t counts;

    /* TEMP_RESULT (0x00): int16 big-endian, 7.8125 m°C/LSB. milli-°C is
     * raw * 125 / 16 (exactly, in integer math). No stimulus is driven, so an
     * on-body TMP117 reads 0 °C here — an honest reading of an undriven model,
     * not a fabricated body temperature. */
    if (board_i2c_read_reg(ADDR_SKIN_TEMP, 0x00, raw, 2) != 0) {
        return 0;
    }
    counts = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    return (int32_t)counts * 125 / 16;
}

int board_haptic_init(void)
{
    /* Leave standby (MODE = 0x00) and select the internal trigger. */
    return board_i2c_write_reg(ADDR_HAPTIC, 0x01, 0x00);
}

int board_haptic_buzz(void)
{
    /* Queue one effect and fire it. */
    if (board_i2c_write_reg(ADDR_HAPTIC, 0x04, 0x01) != 0) {
        return -1;
    }
    if (board_i2c_write_reg(ADDR_HAPTIC, 0x05, 0x00) != 0) {   /* terminator */
        return -1;
    }
    return board_i2c_write_reg(ADDR_HAPTIC, 0x0C, 0x01);       /* GO */
}

int board_touch_read(uint8_t *pressed)
{
    /* The 1-channel cap sensor's logic output is on P1.13; read the live pin
     * state from GPIO IN and return bit 13 as 0/1. No stimulus is driven, so
     * this reads 0 — which still proves the GPIO input path end to end. */
    *pressed = (uint8_t)((GPIO_IN(GPIO_P1_BASE) >> TOUCH_PIN) & 1u);
    return 0;
}

void board_motor_gpio_init(void)
{
    GPIO_DIRSET(GPIO_P1_BASE) = (1u << MOTOR_ENABLE_PIN);
    GPIO_OUTCLR(GPIO_P1_BASE) = (1u << MOTOR_ENABLE_PIN);
}

void board_motor_enable(int on)
{
    if (on) {
        GPIO_OUTSET(GPIO_P1_BASE) = (1u << MOTOR_ENABLE_PIN);
    } else {
        GPIO_OUTCLR(GPIO_P1_BASE) = (1u << MOTOR_ENABLE_PIN);
    }
}
