/* Quick-check: read an MPU6050 at 500 Hz and report the actually achieved rate.
 *
 * No-RTOS variant: the loop uses no FreeRTOS scheduling. It is paced by a bare
 * busy-wait on the hardware timer (esp_timer_get_time), and the init settle uses
 * esp_rom_delay_us. (app_main itself is still entered from a FreeRTOS task - that
 * is unavoidable on ESP-IDF - but nothing in the loop touches the scheduler.)
 * Because the loop never yields, the task watchdog is disabled in sdkconfig.defaults.
 *
 * Wiring matches the main firmware: SDA=GPIO21, SCL=GPIO22, external pull-ups.
 * Every second it prints the measured rate, per-read I2C time (avg/min/max), the
 * loop period spread (dt min/max), the read-failure count, and the latest sample.
 *
 * To try a different rate, change LOOP_HZ.
 */
#include <stdint.h>
#include <inttypes.h>
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "imu500";

/* --- I2C / MPU6050 (same pins and settings as firmware/main/imu.c) --- */
#define I2C_PORT            I2C_NUM_0
#define I2C_SDA_GPIO        GPIO_NUM_21
#define I2C_SCL_GPIO        GPIO_NUM_22
#define I2C_FREQ_HZ         400000      /* try 100000 if the bus is marginal (weak pull-ups) */
#define MPU6050_ADDR        0x68        /* AD0 = GND; 0x69 if AD0 = VCC */
/* Generous vs the RT firmware's 1 ms: a slow-but-working read (marginal pull-ups)
 * completes instead of being aborted mid-transaction, which is what hangs the bus. */
#define MPU_I2C_TIMEOUT_MS  10

#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CFG        0x1B
#define REG_ACCEL_CFG       0x1C
#define REG_ACCEL_XOUT      0x3B        /* first of 14 data bytes */
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

#define ACCEL_LSB_PER_G     16384.0f    /* +/-2 g range */
#define GYRO_LSB_PER_DPS    131.0f      /* +/-250 dps range */

/* Test loop rate. */
#define LOOP_HZ             500
#define LOOP_PERIOD_US      (1000000 / LOOP_HZ)

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_mpu;

static esp_err_t mpu_read(uint8_t reg, uint8_t *dst, size_t n)
{
    return i2c_master_transmit_receive(s_mpu, &reg, 1, dst, n, MPU_I2C_TIMEOUT_MS);
}

static esp_err_t mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_mpu, b, sizeof(b), MPU_I2C_TIMEOUT_MS);
}

static bool mpu_init(void)
{
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(s_bus, &dev, &s_mpu) != ESP_OK) {
        ESP_LOGE(TAG, "failed to add device at 0x%02X", MPU6050_ADDR);
        return false;
    }

    uint8_t who = 0;
    if (mpu_read(REG_WHO_AM_I, &who, 1) != ESP_OK) {
        ESP_LOGE(TAG, "no response (check wiring/power/pull-ups)");
        return false;
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expect 0x68)", who);

    mpu_write(REG_PWR_MGMT_1, 0x00);           /* wake, internal 8 MHz clock */
    esp_rom_delay_us(10000);                    /* 10 ms settle, no scheduler */
    mpu_write(REG_SMPLRT_DIV, 0x00);           /* 1 kHz sample rate */
    mpu_write(REG_CONFIG,     0x03);           /* DLPF ~44 Hz */
    mpu_write(REG_GYRO_CFG,   0x00);           /* +/-250 dps */
    mpu_write(REG_ACCEL_CFG,  0x00);           /* +/-2 g */

    ESP_LOGI(TAG, "MPU6050 ready (1 kHz, DLPF ~44 Hz, +/-2 g, +/-250 dps)");
    return true;
}

void app_main(void)
{
    /* The driver re-reserves the pins on every post-timeout bus reset and warns
     * they are "already reserved" - benign noise. Our fail counter tracks the real
     * failures, so quiet it to keep the stats readable. */
    esp_log_level_set("i2c.common", ESP_LOG_ERROR);

    i2c_master_bus_config_t bus = {
        .i2c_port          = I2C_PORT,
        .sda_io_num        = I2C_SDA_GPIO,
        .scl_io_num        = I2C_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, /* external pull-ups on SDA/SCL */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus, &s_bus));
    ESP_LOGI(TAG, "I2C @ %d Hz on SDA=%d SCL=%d", I2C_FREQ_HZ, I2C_SDA_GPIO, I2C_SCL_GPIO);

    if (!mpu_init()) {
        ESP_LOGE(TAG, "init failed - halting");
        return;
    }
    ESP_LOGI(TAG, "reading IMU at %d Hz (no-RTOS busy-wait); stats every 1 s", LOOP_HZ);

    int64_t next_due  = esp_timer_get_time();
    int64_t win_start = next_due;
    int64_t last      = next_due;
    bool    primed    = false;    /* skip the dt stat on the very first read */

    uint32_t n = 0, fails = 0;
    int32_t  read_min = INT32_MAX, read_max = 0;
    int64_t  read_sum = 0;
    int32_t  dt_min = INT32_MAX, dt_max = 0;
    float    ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, temp = 0;

    for (;;) {
        /* Pace to the next 500 Hz boundary by spinning on the hardware timer.
         * Phase-locked accumulator (no drift), no scheduler involvement. */
        next_due += LOOP_PERIOD_US;
        while (esp_timer_get_time() < next_due) { /* busy-wait */ }

        int64_t t0 = esp_timer_get_time();
        int32_t dt = (int32_t)(t0 - last);
        last = t0;

        uint8_t b[14];
        esp_err_t err  = mpu_read(REG_ACCEL_XOUT, b, sizeof(b));
        int32_t read_us = (int32_t)(esp_timer_get_time() - t0);

        if (err == ESP_OK) {
            ax   = (int16_t)((b[0]  << 8) | b[1])  / ACCEL_LSB_PER_G;
            ay   = (int16_t)((b[2]  << 8) | b[3])  / ACCEL_LSB_PER_G;
            az   = (int16_t)((b[4]  << 8) | b[5])  / ACCEL_LSB_PER_G;
            temp = (int16_t)((b[6]  << 8) | b[7])  / 340.0f + 36.53f;
            gx   = (int16_t)((b[8]  << 8) | b[9])  / GYRO_LSB_PER_DPS;
            gy   = (int16_t)((b[10] << 8) | b[11]) / GYRO_LSB_PER_DPS;
            gz   = (int16_t)((b[12] << 8) | b[13]) / GYRO_LSB_PER_DPS;
        } else {
            fails++;
        }

        if (primed) {
            if (dt < dt_min) dt_min = dt;
            if (dt > dt_max) dt_max = dt;
        }
        primed = true;
        if (read_us < read_min) read_min = read_us;
        if (read_us > read_max) read_max = read_us;
        read_sum += read_us;
        n++;

        int64_t nowus = esp_timer_get_time();
        if (nowus - win_start >= 1000000) {
            float secs = (float)(nowus - win_start) * 1e-6f;
            ESP_LOGI(TAG,
                     "rate=%.1f Hz (%" PRIu32 " reads, %" PRIu32 " fail) | "
                     "read us avg=%.1f min=%ld max=%ld | dt us min=%ld max=%ld | "
                     "a=[%.2f %.2f %.2f]g g=[%.1f %.1f %.1f]dps %.1fC",
                     n / secs, n, fails,
                     (float)read_sum / (float)n, (long)read_min, (long)read_max,
                     (long)dt_min, (long)dt_max,
                     ax, ay, az, gx, gy, gz, temp);

            win_start = nowus;
            n = 0; fails = 0;
            read_min = INT32_MAX; read_max = 0; read_sum = 0;
            dt_min = INT32_MAX; dt_max = 0;
        }
    }
}
