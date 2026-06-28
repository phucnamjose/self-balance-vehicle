#include "control.h"

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "telemetry.h"
#include "imu.h"
#include "motors.h"
#include "encoders.h"
#include "web_server.h"

static const char *TAG = "control";

#define TIMER_RES_HZ   1000000                   /* 1 MHz -> 1 tick = 1 us */
#define ALARM_COUNT    (TIMER_RES_HZ / CONTROL_HZ)

static TaskHandle_t s_control_task;

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
    imu_t imu = { 0 };             /* latest IMU sample */
    int64_t pos_last[2] = { 0 };   /* encoder counts at the previous report */

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t now_us = esp_timer_get_time();
        int64_t dt = now_us - last_us;
        last_us = now_us;

        /* Read the IMU every tick. The I2C transfer (~0.4 ms) blocks this task
         * only, and we watch dt_min/dt_max to confirm it doesn't hurt jitter.
         * (later: feed imu into the orientation estimate + PID -> "level") */
        imu = mpu6050_read();

        /* Apply the latest open-loop motor commands (set from the web terminal).
         * (later: the PID output will drive these instead of a manual command.) */
        motor_set(0, motor_cmd_get(0));
        motor_set(1, motor_cmd_get(1));

        /* Track timing quality. */
        if (dt < dt_min) dt_min = dt;
        if (dt > dt_max) dt_max = dt;
        dt_sum += dt;
        ticks++;

        /* Once per second, hand a snapshot to the reporter task and reset the
         * window. xQueueOverwrite never blocks, so the control loop stays lean -
         * the slow logging/networking happens on the other core. */
        if (ticks % CONTROL_HZ == 0) {
            /* The window is exactly 1 s, so delta counts == counts/sec. */
            int64_t posL = encoder_count(0);
            int64_t posR = encoder_count(1);
            telemetry_t snap = {
                .ticks     = ticks,
                .dt_avg_us = dt_sum / CONTROL_HZ,
                .dt_min_us = dt_min,
                .dt_max_us = dt_max,
                .mL        = motor_cmd_get(0),
                .mR        = motor_cmd_get(1),
                .posL      = posL,
                .posR      = posR,
                .velL      = (int32_t)(posL - pos_last[0]),
                .velR      = (int32_t)(posR - pos_last[1]),
                .imu       = imu,
            };
            pos_last[0] = posL;
            pos_last[1] = posR;
            xQueueOverwrite(s_telemetry_q, &snap);
            dt_min = INT64_MAX; dt_max = 0; dt_sum = 0;
        }
    }
}

/* Runs on core 0. Blocks until the control task posts a snapshot, caches it for
 * the HTTP server, and also logs it to the serial console. */
static void reporter_task(void *arg)
{
    telemetry_t snap;
    for (;;) {
        if (xQueueReceive(s_telemetry_q, &snap, portMAX_DELAY) == pdTRUE) {
            /* Cache for the web server. */
            telemetry_set(&snap);

            ESP_LOGI(TAG,
                     "[core %d] rate=%d Hz dt avg=%lld us (min=%lld max=%lld) mot=[%.0f %.0f]%% ticks=%" PRIu32,
                     xPortGetCoreID(), CONTROL_HZ,
                     (long long)snap.dt_avg_us, (long long)snap.dt_min_us,
                     (long long)snap.dt_max_us, snap.mL * 100.0f, snap.mR * 100.0f, snap.ticks);

            ESP_LOGI(TAG,
                     "         enc posL=%lld posR=%lld  vel=[%ld %ld] cnt/s",
                     (long long)snap.posL, (long long)snap.posR,
                     (long)snap.velL, (long)snap.velR);

            if (snap.imu.ok) {
                ESP_LOGI(TAG,
                         "         IMU a=[%+.2f %+.2f %+.2f]g g=[%+.1f %+.1f %+.1f]dps %.1fC",
                         snap.imu.ax, snap.imu.ay, snap.imu.az,
                         snap.imu.gx, snap.imu.gy, snap.imu.gz, snap.imu.temp_c);
            } else {
                ESP_LOGW(TAG, "         IMU read failed");
            }

            /* Stream to any connected WebSocket terminals. */
            if (web_server_streaming()) {
                char json[512];
                telemetry_to_json(json, sizeof(json), "telemetry");
                ws_broadcast(json);
            }
        }
    }
}

void control_start(void)
{
    /* Mailbox between the two tasks (length 1 -> always the latest snapshot). */
    s_telemetry_q = xQueueCreate(1, sizeof(telemetry_t));
    configASSERT(s_telemetry_q);

    /* Reporter on core 0 (PRO_CPU) - shares this core with Wi-Fi. */
    xTaskCreatePinnedToCore(reporter_task, "reporter", 4096, NULL, 4, NULL, 0);

    /* Control loop on core 1 (APP_CPU), high priority. Create it before the
     * timer so the ISR has a valid task handle to notify. */
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL,
                            configMAX_PRIORITIES - 2, &s_control_task, 1);

    start_control_timer();
}
