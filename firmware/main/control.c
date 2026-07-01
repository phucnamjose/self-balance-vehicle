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

/* Scratch buffer for one packed telemetry frame. The largest topic (imu, 9
 * fields) is 4 + SAMPLES_PER_BATCH*9*4 bytes; 4 KB leaves comfortable margin. */
#define REPORT_FRAME_CAP  4096

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

/* Control-task lifecycle. STOP_CONTROL fully stops (deletes) the task - not just
 * suspends it - and parks the motors; START_CONTROL (re)creates it. The GPTimer
 * and peripherals are set up once and reused across restarts. Distinct from the
 * 'stop' motor-test command, which only zeroes the outputs while the task runs. */
static volatile control_mode_t s_mode = STOP_CONTROL;  /* set to START by control_start() */
static gptimer_handle_t  s_timer;        /* created once, kept across stop/start */
static bool              s_periph_ready; /* peripherals init'd once, on first start */
static volatile bool     s_stop_req;     /* ask the loop to exit at a safe point */

/* ISR context: keep it minimal - just wake the control task for the next tick. */
static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx)
{
    BaseType_t high_prio_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_control_task, &high_prio_woken);
    return high_prio_woken == pdTRUE;
}

/* Create + start the GPTimer once, stashing the handle so a restart can just
 * re-start it (rather than allocate another timer/interrupt). */
static void start_control_timer(void)
{
    gptimer_config_t cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RES_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &s_timer));

    gptimer_event_callbacks_t cbs = { .on_alarm = on_timer_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_timer));

    gptimer_alarm_config_t alarm = {
        .reload_count = 0,
        .alarm_count  = ALARM_COUNT,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_timer, &alarm));
    ESP_ERROR_CHECK(gptimer_start(s_timer));
}

/* High-priority task woken by the timer ISR every 1/CONTROL_HZ seconds. Records
 * one sample per tick into the active buffer; every SAMPLES_PER_BATCH samples it
 * swaps buffers and notifies the reporter task with the full buffer's index. */
static void control_task(void *arg)
{
    /* Claim this task's handle for the ISR before the timer can fire. */
    s_control_task = xTaskGetCurrentTaskHandle();

    /* First start only: bring up every peripheral the control loop drives from
     * THIS task so their interrupts are allocated on core 1 (esp_intr_alloc pins
     * to the calling core): the PCNT rollover ISR (encoders) and the I2C
     * completion ISR the IMU read blocks on. That keeps the whole real-time path
     * insulated from core 0, so a busy Wi-Fi/WebSocket can never delay a tick.
     * On a restart the peripherals already exist, so we skip re-init. (LED init
     * stays on core 0 in app_main - it's only for the boot heartbeat.) */
    if (!s_periph_ready) {
        encoder_init();   /* PCNT quadrature counters, count from 0 */
        motor_init();     /* LEDC PWM channels; motors start stopped */
        i2c_init();
        i2c_scan();
        if (!mpu6050_init()) {
            ESP_LOGW(TAG, "continuing without IMU - check wiring at SDA=21 SCL=22");
        }
        s_periph_ready = true;
    }

    /* Start the GPTimer from THIS task so esp_intr_alloc() pins the timer
     * interrupt to core 1. The first start creates it; a restart just re-starts
     * the existing timer (it was stopped when the task last exited). */
    if (s_timer == NULL) {
        start_control_timer();
    } else {
        ESP_ERROR_CHECK(gptimer_start(s_timer));
    }

    imu_t imu = { 0 };             /* latest IMU sample */
    int64_t pos_last[2] = { 0 };   /* encoder counts at the previous tick */
    int64_t last_us = esp_timer_get_time();   /* timestamp of the previous tick */
    bool    primed = false;        /* skip the warning check on the first tick */
    int     active = 0;            /* buffer we are currently writing into */
    int     idx = 0;               /* next slot in the active buffer */

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* STOP_CONTROL requested: leave the loop here - a safe point, never
         * mid-I2C - and fall through to the cleanup + self-delete below. */
        if (s_stop_req) break;

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
         * The 'stop' command zeroes these; STOP_CONTROL deletes this task
         * entirely. (later: the PID output drives these instead.) */
        float cmdL = motor_cmd_get(0);
        float cmdR = motor_cmd_get(1);
        motor_set(0, cmdL);
        motor_set(1, cmdR);

        /* Per-tick wheel angle (rad) and angular speed (rad/s). Speed is a
         * single-tick finite difference of the counts divided by the ACTUAL
         * elapsed time (1e6/dt_us counts/sec), so a jittered tick doesn't bias
         * the estimate; encoder_cps_to_radps turns counts/sec into rad/s. The
         * first tick and any nonsensical dt fall back to the nominal rate. */
        float cps_scale = (dt_us > 0) ? (1e6f / (float)dt_us) : (float)CONTROL_HZ;
        int64_t posL_cnt = encoder_count(0);
        int64_t posR_cnt = encoder_count(1);
        sample_t *smp = &s_buf[active][idx];
        smp->t_us = now_us;
        smp->posL = (float)encoder_angle_rad(0);
        smp->posR = (float)encoder_angle_rad(1);
        smp->velL = encoder_cps_to_radps((int32_t)((posL_cnt - pos_last[0]) * cps_scale));
        smp->velR = encoder_cps_to_radps((int32_t)((posR_cnt - pos_last[1]) * cps_scale));
        smp->mL   = cmdL;
        smp->mR   = cmdR;
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

    /* Reached only on a STOP_CONTROL request. Stop the timer so the ISR can't
     * notify a deleted task, park the motors, then delete ourselves. A restart
     * (START_CONTROL) creates a fresh task that re-starts the same timer. */
    ESP_ERROR_CHECK(gptimer_stop(s_timer));
    motor_set(0, 0.0f);
    motor_set(1, 0.0f);
    s_control_task = NULL;
    s_stop_req     = false;
    s_mode         = STOP_CONTROL;
    vTaskDelete(NULL);
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

        /* Pack one binary frame per telemetry topic and stream it to any
         * connected WebSocket clients so the page can record each to its own
         * file. The scratch buffer is static (a batch is too big for the task
         * stack, and a static buffer avoids the heap churn that fragmented RAM
         * when this was a per-batch malloc). Only reporter_task touches it. */
        if (web_server_streaming()) {
            static uint8_t frame[REPORT_FRAME_CAP];
            for (int t = 0; t < telemetry_topic_count(); t++) {
                int n = telemetry_topic_pack(frame, sizeof(frame), t, batch,
                                             SAMPLES_PER_BATCH);
                if (n > 0) {
                    ws_broadcast_bin(frame, (size_t)n);
                } else {
                    ESP_LOGW(TAG, "topic '%s' frame did not fit in %u bytes",
                             telemetry_topic_name(t), (unsigned)sizeof(frame));
                }
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

    /* Reporter on core 0 (PRO_CPU) - shares this core with Wi-Fi. If Wi-Fi or a
     * WebSocket client stalls, only this task waits; the control loop is unaffected. */
    xTaskCreatePinnedToCore(reporter_task, "reporter", 4096, NULL, 4, NULL, 0);

    /* Start the control task (creates it on core 1, brings up peripherals + the
     * GPTimer there - see control_task). */
    control_set_mode(START_CONTROL);
}

control_mode_t control_mode(void) { return s_mode; }

void control_set_mode(control_mode_t mode)
{
    if (mode == START_CONTROL) {
        if (s_mode == START_CONTROL) return;          /* already running */
        s_stop_req = false;
        s_mode     = START_CONTROL;
        /* Control loop on core 1 (APP_CPU), high priority, so it preempts
         * anything else there and stays isolated from core 0's Wi-Fi. */
        xTaskCreatePinnedToCore(control_task, "control", 4096, NULL,
                                configMAX_PRIORITIES - 2, NULL, 1);
    } else {  /* STOP_CONTROL */
        if (s_mode == STOP_CONTROL) return;           /* already stopped */
        /* Ask the loop to exit at a safe point; it stops the timer, parks the
         * motors and self-deletes. Wait (bounded) until it's actually gone so
         * callers (e.g. the OTA handler) know the task is stopped on return. */
        s_stop_req = true;
        for (int i = 0; i < 200 && s_control_task != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        s_mode = STOP_CONTROL;
    }
}
