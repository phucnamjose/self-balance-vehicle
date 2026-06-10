/**
 * @file app_main.c
 * @brief Phase 1 / Step 6 - I2C: scan the bus (find the MPU6050).
 *
 * We keep the deterministic timing + PWM structure from Steps 4-5:
 *
 *     GPTimer alarm (hardware) --ISR--> task notification --> control_task
 *     control_task computes a duty each tick -> LEDC PWM
 *
 * and now add an I2C master bus and a boot-time scan that probes every address
 * to report which devices are connected. Next step we'll actually read the
 * MPU6050 over this same bus.
 *
 * What this teaches:
 *   - Setting up an I2C master bus (SDA/SCL pins, internal pull-ups, speed).
 *   - "Scanning": probing each 7-bit address and seeing who ACKs.
 *   - Confirming wiring before writing a full sensor driver.
 *
 * Wiring the MPU6050 (GY-521 module has its own pull-ups):
 *   VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22, AD0 -> GND (addr 0x68).
 *
 * Pins: PWM output on GPIO2 (onboard LED on most DevKits; change if needed).
 */
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "balance_bot";

/* --- I2C bus (MPU6050 lives here) --- */
#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_GPIO   GPIO_NUM_21
#define I2C_SCL_GPIO   GPIO_NUM_22
#define I2C_FREQ_HZ    400000

#define PWM_GPIO       GPIO_NUM_2

#define CONTROL_HZ     200                       /* control loop rate */
#define TIMER_RES_HZ   1000000                   /* 1 MHz -> 1 tick = 1 us */
#define ALARM_COUNT    (TIMER_RES_HZ / CONTROL_HZ)

/* --- LEDC (PWM) configuration --- */
#define PWM_MODE       LEDC_LOW_SPEED_MODE
#define PWM_TIMER      LEDC_TIMER_0
#define PWM_CHANNEL    LEDC_CHANNEL_0
#define PWM_RES_BITS   LEDC_TIMER_10_BIT         /* duty range 0..1023 */
#define PWM_DUTY_MAX   ((1 << 10) - 1)
#define PWM_FREQ_HZ    5000                      /* 5 kHz: flicker-free for an LED.
                                                  * For real motors we'll use ~20 kHz. */

/* Triangle-wave sweep: full fade up+down over this many control ticks. */
#define FADE_PERIOD_TICKS  (CONTROL_HZ * 2)      /* ~2 s up, then handled by mirror */

static TaskHandle_t s_control_task;
static i2c_master_bus_handle_t s_i2c_bus;   /* kept for the next step (reading the IMU) */

/* ISR context: keep it minimal - just wake the control task for the next tick. */
static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx)
{
    BaseType_t high_prio_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_control_task, &high_prio_woken);
    return high_prio_woken == pdTRUE;
}

static void pwm_init(void)
{
    /* 1) The LEDC timer sets the PWM frequency and duty resolution. */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .duty_resolution = PWM_RES_BITS,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    /* 2) The LEDC channel binds a GPIO to that timer and holds the duty value. */
    ledc_channel_config_t ch_cfg = {
        .gpio_num   = PWM_GPIO,
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
}

/* Set PWM duty from a normalised level in [0.0, 1.0]. This is the interface the
 * controller will use later (level = commanded motor effort). */
static void pwm_set_level(float level)
{
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;
    uint32_t duty = (uint32_t)(level * PWM_DUTY_MAX + 0.5f);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

static void i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_PORT,
        .sda_io_num        = I2C_SDA_GPIO,
        .scl_io_num        = I2C_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* helps if the module lacks pull-ups */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));
    ESP_LOGI(TAG, "I2C master on SDA=%d SCL=%d @ %d Hz",
             I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ);
}

/* Probe every 7-bit address and print which ones acknowledge. A device that is
 * present pulls the bus low to ACK its address, so i2c_master_probe() succeeds. */
static void i2c_scan(void)
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

static void start_control_timer(void)
{
    gptimer_handle_t timer = NULL;
    gptimer_config_t cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RES_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &timer));

    gptimer_event_callbacks_t cbs = { .on_alarm = on_timer_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer));

    gptimer_alarm_config_t alarm = {
        .reload_count = 0,
        .alarm_count  = ALARM_COUNT,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm));
    ESP_ERROR_CHECK(gptimer_start(timer));
}

/* High-priority task woken by the timer ISR every 1/CONTROL_HZ seconds. */
static void control_task(void *arg)
{
    int64_t last_us = esp_timer_get_time();
    int64_t dt_min = INT64_MAX, dt_max = 0, dt_sum = 0;
    uint32_t ticks = 0;
    uint32_t phase = 0;            /* 0..2*FADE_PERIOD_TICKS-1 for up/down sweep */

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t now_us = esp_timer_get_time();
        int64_t dt = now_us - last_us;
        last_us = now_us;

        /* ---- (later: read IMU, run PID -> "level" here) ---- */

        /* Triangle wave: ramp 0->1 over FADE_PERIOD_TICKS, then 1->0. */
        float level;
        if (phase < FADE_PERIOD_TICKS) {
            level = (float)phase / FADE_PERIOD_TICKS;
        } else {
            level = 1.0f - (float)(phase - FADE_PERIOD_TICKS) / FADE_PERIOD_TICKS;
        }
        if (++phase >= 2 * FADE_PERIOD_TICKS) phase = 0;

        pwm_set_level(level);

        /* Track timing quality. */
        if (dt < dt_min) dt_min = dt;
        if (dt > dt_max) dt_max = dt;
        dt_sum += dt;
        ticks++;

        if (ticks % CONTROL_HZ == 0) {
            int64_t avg = dt_sum / CONTROL_HZ;
            ESP_LOGI(TAG, "rate=%d Hz dt avg=%lld us (min=%lld max=%lld) duty=%.0f%%",
                     CONTROL_HZ, (long long)avg, (long long)dt_min,
                     (long long)dt_max, level * 100.0f);
            dt_min = INT64_MAX; dt_max = 0; dt_sum = 0;
        }
    }
}

void app_main(void)
{
    printf("\n=== Self-Balancing Vehicle - I2C scan + PWM on the control tick ===\n");
    ESP_LOGI(TAG, "GPTimer @ %d Hz -> control_task -> LEDC PWM fade on GPIO%d",
             CONTROL_HZ, PWM_GPIO);

    pwm_init();

    /* Bring up I2C and scan once at boot so we can confirm the IMU is wired. */
    i2c_init();
    i2c_scan();

    /* Create the control task first so the ISR has a valid handle to notify. */
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL,
                            configMAX_PRIORITIES - 2, &s_control_task, 1);

    start_control_timer();
}
