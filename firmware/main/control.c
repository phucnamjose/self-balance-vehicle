#include "control.h"

#include <inttypes.h>
#include <stdlib.h>
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

/* Double buffer of per-tick samples. control_task fills one half while the
 * reporter drains the other; they swap every SAMPLES_PER_BATCH samples. The
 * queue hands the reporter the index (0/1) of whichever buffer just filled. */
static sample_t      s_buf[2][SAMPLES_PER_BATCH];
static QueueHandle_t s_ready_q;

/* Timing-warning handoff. The control loop checks each tick's period and, if it
 * runs long or short, records the offending value and bumps the counter. The
 * reporter (off the real-time path) drains these into a single warning per
 * batch, so logging/Wi-Fi work never runs inside the control loop. */
static volatile int32_t  s_warn_dt_us;   /* most recent out-of-range period */
static volatile uint32_t s_warn_count;   /* violations since the last report */

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

/* High-priority task woken by the timer ISR every 1/CONTROL_HZ seconds. Records
 * one sample per tick into the active buffer; every SAMPLES_PER_BATCH samples it
 * swaps buffers and notifies the reporter task with the full buffer's index. */
static void control_task(void *arg)
{
    imu_t imu = { 0 };             /* latest IMU sample */
    int64_t pos_last[2] = { 0 };   /* encoder counts at the previous tick */
    int64_t last_us = esp_timer_get_time();   /* timestamp of the previous tick */
    bool    primed = false;        /* skip the warning check on the first tick */
    int     active = 0;            /* buffer we are currently writing into */
    int     idx = 0;               /* next slot in the active buffer */

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t now_us = esp_timer_get_time();
        int32_t dt_us  = (int32_t)(now_us - last_us);
        last_us = now_us;

        /* Check this tick's period right here in the loop. Out-of-range periods
         * only stash a value for the reporter to announce - no logging here. */
        if (primed && (dt_us > DT_MAX_WARN_US || dt_us < DT_MIN_WARN_US)) {
            s_warn_dt_us = dt_us;
            s_warn_count++;
        }
        primed = true;

        /* Read the IMU every tick. The I2C transfer (~0.4 ms) blocks this task
         * only, and the per-tick dt check above flags it if jitter creeps in.
         * (later: feed imu into the orientation estimate + PID -> "level") */
        imu = mpu6050_read();

        /* Apply the latest open-loop motor commands (set from the web terminal).
         * (later: the PID output will drive these instead of a manual command.) */
        motor_set(0, motor_cmd_get(0));
        motor_set(1, motor_cmd_get(1));

        /* Per-tick wheel angle (rad) and angular speed (rad/s). Speed is a
         * single-tick finite difference of the counts: delta * CONTROL_HZ gives
         * counts/sec, which encoder_cps_to_radps turns into rad/s. */
        int64_t posL_cnt = encoder_count(0);
        int64_t posR_cnt = encoder_count(1);
        sample_t *smp = &s_buf[active][idx];
        smp->t_us = now_us;
        smp->posL = (float)encoder_angle_rad(0);
        smp->posR = (float)encoder_angle_rad(1);
        smp->velL = encoder_cps_to_radps((int32_t)((posL_cnt - pos_last[0]) * CONTROL_HZ));
        smp->velR = encoder_cps_to_radps((int32_t)((posR_cnt - pos_last[1]) * CONTROL_HZ));
        smp->mL   = motor_cmd_get(0);
        smp->mR   = motor_cmd_get(1);
        smp->imu  = imu;
        pos_last[0] = posL_cnt;
        pos_last[1] = posR_cnt;

        /* Buffer full: hand it to the reporter and switch to the other half.
         * xQueueSend with timeout 0 never blocks the control loop - if the
         * reporter fell behind we drop this batch rather than stall the loop. */
        if (++idx >= SAMPLES_PER_BATCH) {
            if (xQueueSend(s_ready_q, &active, 0) != pdPASS) {
                ESP_LOGW(TAG, "reporter behind - dropping a telemetry batch");
            }
            active = !active;
            idx = 0;
        }
    }
}

/* Announce any timing violations the control loop flagged since the last batch,
 * to both the serial console and the web terminal. Coalesces a burst into one
 * message with a count so we never spam. */
static void emit_pending_warning(void)
{
    uint32_t n = s_warn_count;
    if (n == 0) return;
    int32_t dt = s_warn_dt_us;
    s_warn_count = 0;            /* clear before reporting; races are harmless */

    char msg[160];
    snprintf(msg, sizeof(msg),
             "loop timing out of range: %" PRIu32 " tick(s) in the last %d ms, "
             "latest dt=%ld us (want ~%d, limits %d..%d)",
             n, 1000 * SAMPLES_PER_BATCH / CONTROL_HZ, (long)dt,
             CONTROL_PERIOD_US, DT_MIN_WARN_US, DT_MAX_WARN_US);
    ESP_LOGW(TAG, "%s", msg);
    char json[256];
    snprintf(json, sizeof(json), "{\"type\":\"warn\",\"text\":\"%s\"}", msg);
    ws_broadcast(json);
}

/* Runs on core 0. Blocks until the control task hands over a full buffer, then:
 * emits any pending timing warning, caches the latest sample for the web
 * server, and streams the whole batch to connected clients to be saved. */
static void reporter_task(void *arg)
{
    int idx;
    for (;;) {
        if (xQueueReceive(s_ready_q, &idx, portMAX_DELAY) != pdTRUE) continue;

        const sample_t *batch = s_buf[idx];

        /* Cache the most recent sample for /api/telemetry and the stats cmd. */
        telemetry_set_latest(&batch[SAMPLES_PER_BATCH - 1]);

        /* Speak up only if the control loop flagged out-of-range periods. */
        emit_pending_warning();

        /* Stream one frame per telemetry topic to any connected WebSocket
         * clients so the page can record each to its own file. Build them on the
         * heap (reusing one buffer) - a batch is too big for the task stack. */
        if (web_server_streaming()) {
            size_t cap = 24 * 1024;
            char *json = malloc(cap);
            if (json) {
                for (int t = 0; t < telemetry_topic_count(); t++) {
                    int n = telemetry_topic_json(json, cap, t, batch,
                                                 SAMPLES_PER_BATCH);
                    if (n > 0) {
                        ws_broadcast(json);
                    } else {
                        ESP_LOGW(TAG, "topic '%s' JSON did not fit in %u bytes",
                                 telemetry_topic_name(t), (unsigned)cap);
                    }
                }
                free(json);
            } else {
                ESP_LOGW(TAG, "out of memory serializing telemetry batch");
            }
        }
    }
}

void control_start(void)
{
    /* Queue of ready-buffer indices. Length 2 so the control task can hand over
     * a buffer even if the reporter is mid-cycle on the previous one. */
    s_ready_q = xQueueCreate(2, sizeof(int));
    configASSERT(s_ready_q);

    /* Reporter on core 0 (PRO_CPU) - shares this core with Wi-Fi. */
    xTaskCreatePinnedToCore(reporter_task, "reporter", 4096, NULL, 4, NULL, 0);

    /* Control loop on core 1 (APP_CPU), high priority. Create it before the
     * timer so the ISR has a valid task handle to notify. */
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL,
                            configMAX_PRIORITIES - 2, &s_control_task, 1);

    start_control_timer();
}
