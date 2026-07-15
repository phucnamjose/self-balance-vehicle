#include "imu.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "imu";

/* --- I2C bus (MPU6050 lives here) --- */
#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_GPIO   GPIO_NUM_21
#define I2C_SCL_GPIO   GPIO_NUM_22
#define I2C_FREQ_HZ    400000

/* MPU6050 data-ready INT -> this input-only pin (push-pull active-high, no pull needed). */
#define MPU_INT_GPIO   GPIO_NUM_34

/* --- MPU6050 registers (datasheet "Register Map") --- */
#define MPU6050_ADDR        0x68    /* AD0 = GND; 0x69 if AD0 = VCC */
#define MPU_REG_SMPLRT_DIV  0x19
#define MPU_REG_CONFIG      0x1A    /* DLPF setting */
#define MPU_REG_GYRO_CFG    0x1B    /* gyro full-scale range */
#define MPU_REG_ACCEL_CFG   0x1C    /* accel full-scale range */
#define MPU_REG_FIFO_EN     0x23    /* which sensors stream into the FIFO */
#define MPU_REG_INT_PIN_CFG 0x37    /* INT pin electrical behaviour */
#define MPU_REG_INT_ENABLE  0x38    /* interrupt sources */
#define MPU_REG_ACCEL_XOUT  0x3B    /* first of 14 data bytes (accel/temp/gyro) */
#define MPU_REG_USER_CTRL   0x6A    /* FIFO enable / reset */
#define MPU_REG_PWR_MGMT_1  0x6B
#define MPU_REG_FIFO_COUNTH 0x72    /* bytes currently in the FIFO (H:L) */
#define MPU_REG_FIFO_RW     0x74    /* FIFO read/write port */
#define MPU_REG_WHO_AM_I    0x75    /* returns 0x68 on a genuine MPU6050 */

/* Register bit fields we use. */
#define MPU_FIFO_EN_ALL     0xF8    /* accel + temp + gyro X/Y/Z into the FIFO */
#define MPU_USER_FIFO_EN    0x40    /* USER_CTRL: enable the FIFO */
#define MPU_USER_FIFO_RESET 0x04    /* USER_CTRL: clear the FIFO */
#define MPU_INT_DATA_RDY    0x01    /* INT_ENABLE: assert INT on each new sample */

/* One FIFO frame matches the ACCEL_XOUT block: accel(6) + temp(2) + gyro(6). We read
 * whole frames only. Cap a single drain to a few frames; a longer backlog means the
 * task stalled, so we resync (reset + data-register read) rather than a huge burst. */
#define MPU_FIFO_FRAME      14
#define MPU_FIFO_MAXREAD    (MPU_FIFO_FRAME * 8)

/* Sensitivity for the default ranges we configure below (datasheet sec 6.1-6.2):
 *   accel +/-2 g    -> 16384 LSB per g
 *   gyro  +/-250 dps -> 131.0 LSB per deg/s
 * Temperature: degC = raw/340 + 36.53. */
#define MPU_ACCEL_LSB_PER_G    16384.0f
#define MPU_GYRO_LSB_PER_DPS   131.0f

/* Per-read I2C timeout. Must sit ABOVE a normal read (~0.5-1.7 ms under scheduler
 * load), otherwise the driver aborts a transfer mid-transaction - which leaves the
 * MPU holding the bus and triggers a reset + stall cascade (measured 5-25 ms reads).
 * It only bounds a genuinely stuck bus; a healthy read still returns in ~1 ms. The
 * caller holds the last sample on the rare timeout. */
#define MPU_I2C_TIMEOUT_MS     5

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_mpu;       /* MPU6050 device on the I2C bus */
static bool s_mpu_ok;                       /* did the IMU init succeed? */
static bool s_int_installed;                /* GPIO34 data-ready ISR added once */

/* Data-ready edge count, bumped by the GPIO34 ISR on every MPU sample (500 Hz). The
 * IMU task diffs it each tick to learn how many new samples arrived (skip count) and
 * whether the IMU is alive - all without an I2C transaction. */
static volatile uint32_t s_dr_count;

/* Gyro zero-rate bias (deg/s), subtracted from each read. Starts at 0 (raw);
 * gyrocal (control.c) measures it at rest and stores it here. */
static volatile float s_gyro_bias[3];       /* [gx, gy, gz] */

/* Minimal ISR: just count the data-ready edge. */
static void IRAM_ATTR mpu_int_isr(void *arg) { s_dr_count++; }

uint32_t mpu6050_dr_count(void) { return s_dr_count; }

void imu_set_gyro_bias(float bx, float by, float bz)
{
    s_gyro_bias[0] = bx;
    s_gyro_bias[1] = by;
    s_gyro_bias[2] = bz;
}

void imu_get_gyro_bias(float *bx, float *by, float *bz)
{
    if (bx) *bx = s_gyro_bias[0];
    if (by) *by = s_gyro_bias[1];
    if (bz) *bz = s_gyro_bias[2];
}

void i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_PORT,
        .sda_io_num        = I2C_SDA_GPIO,
        .scl_io_num        = I2C_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,   /* external pull-ups on SDA/SCL */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));
    ESP_LOGI(TAG, "I2C master on SDA=%d SCL=%d @ %d Hz",
             I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ);
}

/* Probe every 7-bit address and print which ones acknowledge. A device that is
 * present pulls the bus low to ACK its address, so i2c_master_probe() succeeds. */
void i2c_scan(void)
{
    ESP_LOGI(TAG, "scanning I2C bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        esp_err_t err = i2c_master_probe(s_i2c_bus, addr, 50 /* ms */);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  found device at 0x%02X%s", addr,
                     (addr == 0x68 || addr == 0x69) ? "  <- looks like MPU6050" : "");
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "  no devices found - check wiring, power and pull-ups");
    } else {
        ESP_LOGI(TAG, "scan complete: %d device(s)", found);
    }
}

/* Write one byte to an MPU register: send [reg, value] in a single transaction. */
static esp_err_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_mpu, buf, sizeof(buf), MPU_I2C_TIMEOUT_MS);
}

/* Read @p n bytes starting at @p reg: write the register pointer, then read.
 * i2c_master_transmit_receive does the write+repeated-start+read in one call. */
static esp_err_t mpu_read_regs(uint8_t reg, uint8_t *dst, size_t n)
{
    return i2c_master_transmit_receive(s_mpu, &reg, 1, dst, n, MPU_I2C_TIMEOUT_MS);
}

bool mpu6050_init(void)
{
    /* Idempotent for retry: add the bus device only once (a second add of the same
     * address fails), then re-probe and re-configure on every call. */
    if (s_mpu == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = MPU6050_ADDR,
            .scl_speed_hz    = I2C_FREQ_HZ,
        };
        if (i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_mpu) != ESP_OK) {
            ESP_LOGE(TAG, "MPU6050: failed to add device at 0x%02X", MPU6050_ADDR);
            s_mpu = NULL;
            return false;
        }
    }

    s_mpu_ok = false;   /* clear until this attempt fully succeeds */

    uint8_t who = 0;
    if (mpu_read_regs(MPU_REG_WHO_AM_I, &who, 1) != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050: no response (check wiring/power)");
        return false;
    }
    ESP_LOGI(TAG, "MPU6050 WHO_AM_I = 0x%02X (expect 0x68)", who);

    /* PWR_MGMT_1 = 0 clears the SLEEP bit and selects the internal 8 MHz clock,
     * so the sensor starts converting. */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    mpu_write_reg(MPU_REG_SMPLRT_DIV, 0x01);   /* sample rate = 1 kHz / (1+1) = 500 Hz */
    mpu_write_reg(MPU_REG_CONFIG,     0x03);   /* DLPF ~44 Hz accel / ~42 Hz gyro */
    mpu_write_reg(MPU_REG_GYRO_CFG,   0x00);   /* +/-250 dps */
    mpu_write_reg(MPU_REG_ACCEL_CFG,  0x00);   /* +/-2 g */

    /* Data-ready INT: active-high push-pull pulse on the INT pin, cleared on read
     * (INT_PIN_CFG = 0x00). Drives the GPIO34 edge counter below. */
    mpu_write_reg(MPU_REG_INT_PIN_CFG, 0x00);
    mpu_write_reg(MPU_REG_INT_ENABLE,  MPU_INT_DATA_RDY);

    /* Stream accel+temp+gyro into the FIFO and clear it, so the IMU task can read the
     * newest buffered frame. Reset with FIFO disabled, then enable (datasheet order). */
    mpu_write_reg(MPU_REG_FIFO_EN,   MPU_FIFO_EN_ALL);
    mpu_write_reg(MPU_REG_USER_CTRL, MPU_USER_FIFO_RESET);
    mpu_write_reg(MPU_REG_USER_CTRL, MPU_USER_FIFO_EN);

    /* GPIO34 rising-edge ISR counts data-ready pulses. Installed once (idempotent
     * across IMU-retry); the pin is input-only with no internal pull (external none
     * needed for the push-pull INT). */
    if (!s_int_installed) {
        gpio_config_t int_io = {
            .pin_bit_mask = (1ULL << MPU_INT_GPIO),
            .mode         = GPIO_MODE_INPUT,
            .intr_type    = GPIO_INTR_POSEDGE,
        };
        ESP_ERROR_CHECK(gpio_config(&int_io));
        esp_err_t isr = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (isr != ESP_OK && isr != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(isr);
        ESP_ERROR_CHECK(gpio_isr_handler_add(MPU_INT_GPIO, mpu_int_isr, NULL));
        s_int_installed = true;
    }

    s_mpu_ok = true;
    ESP_LOGI(TAG, "MPU6050 ready (500 Hz, +/-2 g, +/-250 dps, DLPF on, FIFO+INT on GPIO%d)",
             MPU_INT_GPIO);
    return true;
}

/* Convert a 14-byte accel/temp/gyro block to physical units. Each axis is a big-endian
 * signed 16-bit value: [0..5] accel X,Y,Z  [6..7] temp  [8..13] gyro X,Y,Z. */
static void mpu_parse(const uint8_t *b, imu_t *s)
{
    int16_t ax = (int16_t)((b[0]  << 8) | b[1]);
    int16_t ay = (int16_t)((b[2]  << 8) | b[3]);
    int16_t az = (int16_t)((b[4]  << 8) | b[5]);
    int16_t t  = (int16_t)((b[6]  << 8) | b[7]);
    int16_t gx = (int16_t)((b[8]  << 8) | b[9]);
    int16_t gy = (int16_t)((b[10] << 8) | b[11]);
    int16_t gz = (int16_t)((b[12] << 8) | b[13]);

    s->ax = ax / MPU_ACCEL_LSB_PER_G;
    s->ay = ay / MPU_ACCEL_LSB_PER_G;
    s->az = az / MPU_ACCEL_LSB_PER_G;
    s->gx = gx / MPU_GYRO_LSB_PER_DPS - s_gyro_bias[0];
    s->gy = gy / MPU_GYRO_LSB_PER_DPS - s_gyro_bias[1];
    s->gz = gz / MPU_GYRO_LSB_PER_DPS - s_gyro_bias[2];
    s->temp_c = t / 340.0f + 36.53f;
    s->ok = true;
}

imu_t mpu6050_read(void)
{
    imu_t s = { 0 };
    if (!s_mpu_ok) return s;
    uint8_t b[14];
    if (mpu_read_regs(MPU_REG_ACCEL_XOUT, b, sizeof(b)) != ESP_OK) return s;
    mpu_parse(b, &s);
    return s;
}

bool mpu6050_read_newest(imu_t *out)
{
    if (!s_mpu_ok) return false;

    /* How many whole frames are buffered. (The INT counter tells the caller how many
     * samples arrived; this read only needs the byte count to drain to the newest.) */
    uint8_t cnt[2];
    if (mpu_read_regs(MPU_REG_FIFO_COUNTH, cnt, 2) != ESP_OK) return false;
    uint16_t n = ((uint16_t)cnt[0] << 8) | cnt[1];

    /* Empty (INT/FIFO desync), overflowed, or a long backlog: resync by clearing the
     * FIFO and reading the data registers, which always hold the newest conversion. */
    if (n < MPU_FIFO_FRAME || n > MPU_FIFO_MAXREAD || (n % MPU_FIFO_FRAME) != 0) {
        mpu_write_reg(MPU_REG_USER_CTRL, MPU_USER_FIFO_RESET);
        mpu_write_reg(MPU_REG_USER_CTRL, MPU_USER_FIFO_EN);
        *out = mpu6050_read();
        return out->ok;
    }

    /* Drain the pending frames in one burst and keep the last (newest); older frames
     * are skipped. */
    uint8_t buf[MPU_FIFO_MAXREAD];
    if (mpu_read_regs(MPU_REG_FIFO_RW, buf, n) != ESP_OK) return false;
    mpu_parse(&buf[n - MPU_FIFO_FRAME], out);
    return true;
}
