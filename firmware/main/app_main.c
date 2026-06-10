/**
 * @file app_main.c
 * @brief Phase 1 / Step 7 - Multi-tasking across both ESP32 cores.
 *
 * Architecture (the split we're building toward):
 *
 *     Core 1 (APP_CPU):  control_task  - hard real-time loop, no slow work
 *           GPTimer ISR --notify--> control_task --> LEDC PWM
 *           control_task --(1-slot queue)--> telemetry snapshot
 *
 *     Core 0 (PRO_CPU):  reporter_task - consumes telemetry and logs it
 *           (this is where Wi-Fi + a web server will live in Phase 2)
 *
 * What this teaches:
 *   - Running multiple FreeRTOS tasks, each pinned to a specific core.
 *   - Keeping the real-time loop lean by moving slow work (logging, later
 *     networking) to another task/core.
 *   - Passing data between tasks safely with a queue used as a "mailbox"
 *     (length 1 + xQueueOverwrite = always holds the latest snapshot).
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
#include "freertos/queue.h"
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

/* Snapshot of loop state, produced by control_task and consumed by reporter_task. */
typedef struct {
    uint32_t ticks;        /* total control ticks so far */
    int64_t  dt_avg_us;    /* average loop period over the last window */
    int64_t  dt_min_us;    /* min/max loop period (jitter) over the window */
    int64_t  dt_max_us;
    float    duty;         /* last PWM level [0..1] */
} telemetry_t;

/* 1-slot queue used as a mailbox: control_task overwrites it with the latest
 * snapshot; reporter_task blocks until a new one arrives. */
static QueueHandle_t s_telemetry_q;

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

        /* Once per second, hand a snapshot to the reporter task and reset the
         * window. xQueueOverwrite never blocks, so the control loop stays lean -
         * the slow logging/networking happens on the other core. */
        if (ticks % CONTROL_HZ == 0) {
            telemetry_t snap = {
                .ticks     = ticks,
                .dt_avg_us = dt_sum / CONTROL_HZ,
                .dt_min_us = dt_min,
                .dt_max_us = dt_max,
                .duty      = level,
            };
            xQueueOverwrite(s_telemetry_q, &snap);
            dt_min = INT64_MAX; dt_max = 0; dt_sum = 0;
        }
    }
}

/* Runs on core 0. Blocks until the control task posts a snapshot, then logs it.
 * In Phase 2 this task (or a sibling on core 0) will serve the data over Wi-Fi. */
static void reporter_task(void *arg)
{
    telemetry_t snap;
    for (;;) {
        if (xQueueReceive(s_telemetry_q, &snap, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG,
                     "[core %d] rate=%d Hz dt avg=%lld us (min=%lld max=%lld) duty=%.0f%% ticks=%" PRIu32,
                     xPortGetCoreID(), CONTROL_HZ,
                     (long long)snap.dt_avg_us, (long long)snap.dt_min_us,
                     (long long)snap.dt_max_us, snap.duty * 100.0f, snap.ticks);
        }
    }
}

void app_main(void)
{
    printf("\n=== Self-Balancing Vehicle - dual-core multitasking ===\n");
    ESP_LOGI(TAG, "core 1 = control loop, core 0 = reporter (Wi-Fi later)");

    pwm_init();

    /* Bring up I2C and scan once at boot so we can confirm the IMU is wired. */
    i2c_init();
    i2c_scan();

    /* Mailbox between the two tasks (length 1 -> always the latest snapshot). */
    s_telemetry_q = xQueueCreate(1, sizeof(telemetry_t));
    configASSERT(s_telemetry_q);

    /* Reporter on core 0 (PRO_CPU) - shares this core with Wi-Fi later. */
    xTaskCreatePinnedToCore(reporter_task, "reporter", 4096, NULL, 4, NULL, 0);

    /* Control loop on core 1 (APP_CPU), high priority. Create it before the
     * timer so the ISR has a valid task handle to notify. */
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL,
                            configMAX_PRIORITIES - 2, &s_control_task, 1);

    start_control_timer();
}
