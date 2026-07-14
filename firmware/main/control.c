#include "control.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>
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
#include "wheel_pi.h"
#include "balance_pid.h"
#include "estimator.h"
#include "web_server.h"

static const char *TAG = "control";

#define TIMER_RES_HZ   1000000                   /* 1 MHz -> 1 tick = 1 us */
#define ALARM_COUNT    (TIMER_RES_HZ / CONTROL_HZ)

/* Wheel speed = finite difference of encoder counts over a sliding VEL_WIN-tick
 * window; averaging cuts count-quantization noise (~14 ms window at 500 Hz).
 * A longer window trades resolution for delay - wrong on a latency-limited loop. */
#define VEL_WIN        8

/* Gyro is reported in deg/s; the balance loop wants the tilt rate in rad/s. */
#define DEG2RAD        (float)(M_PI / 180.0)

/* Sub-rate for the IMU read, orientation estimate and balance loop: 250 Hz, a healthy
 * margin above the ~32 rad/s crossover. Pacing the costly MPU6050 read (plus estimator
 * + balance ZOH) at half rate frees budget; the wheel loop, encoders and telemetry keep
 * the full CONTROL_HZ tick. Must divide CONTROL_HZ. */
#define BALANCE_HZ     250
#define BALANCE_DIV    (CONTROL_HZ / BALANCE_HZ)
_Static_assert(CONTROL_HZ % BALANCE_HZ == 0, "BALANCE_HZ must divide CONTROL_HZ exactly");

/* Consecutive failed IMU reads that mean it is gone (~200 ms at 500 Hz), not a blip. */
#define IMU_FAIL_LIMIT 100

/* Scratch buffer for one packed telemetry frame, derived from SAMPLES_PER_BATCH so
 * it tracks CONTROL_HZ (a fixed size overflowed at 500 Hz and dropped topics).
 * Keep REPORT_FRAME_MAX_FIELDS >= the widest topic's field count. +64 B slack. */
#define REPORT_FRAME_MAX_FIELDS 9
#define REPORT_FRAME_CAP  (4 + SAMPLES_PER_BATCH * (8 + (REPORT_FRAME_MAX_FIELDS - 1) * 4) + 64)

/* Stream the bulky imu topic once per second (every IMU_STREAM_DIV-th batch = batches
 * per second) while motors/angles stream every batch. */
#define IMU_STREAM_DIV  (REPORT_HZ / SAMPLES_PER_BATCH)

static TaskHandle_t s_control_task;

/* Double buffer: control_task fills one half while the reporter drains the other,
 * swapping every SAMPLES_PER_BATCH. The queue passes the filled buffer's index. */
static sample_t      s_buf[2][SAMPLES_PER_BATCH];
static QueueHandle_t s_ready_q;

/* Timing-warning handoff: the loop records an out-of-range period and bumps the
 * counter; the reporter (off the RT path) drains it into one warning per batch. */
static volatile int32_t  s_warn_dt_us;   /* most recent out-of-range period */
static volatile uint32_t s_warn_count;   /* violations since the last report */

/* Same handoff for per-tick run time: stash the worst over-budget value and count
 * it. Separate from the period warning so slow-to-schedule vs slow-to-compute is clear. */
static volatile int32_t  s_warn_run_us;    /* worst over-budget run time */
static volatile uint32_t s_warn_run_count; /* overruns since the last report */

/* Telemetry batches dropped because the reporter fell behind. Counted on the RT
 * path (never logged there), drained by the reporter. */
static volatile uint32_t s_drop_count;

/* Per-phase profiling: the IMU read is the tick's only variable-latency blocking
 * call (I2C), so time it separately. s_imu_stale_count counts failed reads. */
static volatile int32_t  s_warn_imu_us;      /* worst IMU-read time in the window */
static volatile uint32_t s_imu_stale_count;  /* failed/timed-out reads (held last) */
static volatile bool     s_imu_lost_evt;     /* set when the IMU is dropped; reporter broadcasts it */

/* Control-task lifecycle: STOP_CONTROL deletes the task and parks the motors;
 * START_CONTROL re-creates it. The GPTimer/peripherals are set up once and reused. */
static volatile control_mode_t s_mode = STOP_CONTROL;  /* set to START by control_start() */
static gptimer_handle_t  s_timer;        /* created once, kept across stop/start */
static bool              s_periph_ready; /* peripherals init'd once, on first start */
static bool              s_imu_present;   /* mpu6050_init() succeeded; if false we never
                                           * touch the I2C bus and balance stays disabled */
static volatile bool     s_stop_req;     /* ask the loop to exit at a safe point */

/* Research feature flags, read every tick. Default (all off) is a plain open-loop
 * motor test - no estimation or controller. */
static volatile bool     s_est_enabled;  /* run the orientation (roll/pitch) estimate */
static volatile bool     s_ctrl_enabled; /* run the wheel/motor controller */
static volatile bool     s_bal_enabled;  /* run the balance (tilt) outer loop */

/* Scripted open-loop playback (TEST_MOTORS): hold (mL,mR) per step for a duration.
 * Durations are stored as cumulative end times. Static buffer so the loop never
 * touches freed memory; the uploader publishes s_play_len last. */
typedef struct { float t_end, mL, mR; } play_pt_t;   /* t_end: step end, s from start */
#define PLAYBACK_MAX 20
static play_pt_t         s_play[PLAYBACK_MAX];
static volatile int      s_play_len;     /* number of loaded steps */
static volatile int      s_play_idx;     /* index of the active step */
static volatile bool     s_play_active;  /* playback running */
static volatile bool     s_play_report;  /* playback finished on its own: reporter announces */
static int64_t           s_play_t0_us;   /* esp_timer time when playback started */

/* Deadband sweep (TEST_MOTORS): ramp both motors 0..DB_MAX forward then reverse and
 * latch the duty where each wheel first turns. Report-only; measures per-wheel,
 * per-direction deadband thresholds for controller bring-up. */
#define DB_MAX          0.15f   /* ramp ceiling (15% duty); wheels usually move well before this */
#define DB_RAMP_RATE    0.01f   /* duty/second: 0 -> DB_MAX over ~15 s for fine resolution */
#define DB_MOVE_COUNTS  5       /* encoder counts (~1.4 deg) that count as "the wheel turned" */
#define DB_SETTLE_S     0.5f    /* park time between the forward and reverse phases */
enum { DB_PHASE_FWD, DB_PHASE_SETTLE, DB_PHASE_REV };   /* sweep phases */
static volatile bool s_db_active;        /* sweep running */
static volatile bool s_db_report;        /* sweep finished: reporter should emit the result */
static int     s_db_phase;               /* current sweep phase (DB_PHASE_*) */
static int64_t s_db_t0_us;               /* esp_timer time the current phase started */
static int64_t s_db_start_cnt[2];        /* encoder counts at the phase start (per wheel) */
static float   s_db_thresh[2][2];        /* latched threshold duty [wheel][0=fwd,1=rev] */
static bool    s_db_found[2][2];         /* whether that wheel/direction has tripped yet */

/* Gyro-bias calibration: average the gyro while held still and fold the mean into
 * the stored bias so the estimator sees zero rate. A web command arms it, the loop
 * runs it over GYROCAL_N ticks, the reporter emits the result. A per-axis spread
 * guard flags motion (bias is orientation-independent but must be motionless). */
#define GYROCAL_N        1000     /* samples to average (~2 s at 500 Hz) */
#define GYROCAL_MOVE_DPS 2.0f     /* per-axis min..max spread that flags "it moved" */
static volatile bool s_gc_active;        /* calibration running */
static volatile bool s_gc_report;        /* finished: reporter should emit the result */
static int    s_gc_n;                    /* samples accumulated so far */
static double s_gc_sum[3];               /* running sum of the (corrected) gyro */
static float  s_gc_min[3], s_gc_max[3];  /* per-axis extremes, for the motion guard */
static float  s_gc_bias[3];              /* resulting absolute bias, dps (for the report) */
static bool   s_gc_moved;                /* spread exceeded the still threshold */

/* Accumulate one gyro sample; on the last, fold the mean into the stored bias
 * unless the robot moved. The gyro is already bias-corrected, so the mean is a
 * residual - re-running refines rather than resets. */
static void gyrocal_step(imu_t imu)
{
    if (!imu.ok) return;   /* skip a bad read; do not advance the counter */
    float g[3] = { imu.gx, imu.gy, imu.gz };
    if (s_gc_n == 0) {
        for (int i = 0; i < 3; i++) { s_gc_sum[i] = 0.0; s_gc_min[i] = s_gc_max[i] = g[i]; }
    }
    for (int i = 0; i < 3; i++) {
        s_gc_sum[i] += g[i];
        if (g[i] < s_gc_min[i]) s_gc_min[i] = g[i];
        if (g[i] > s_gc_max[i]) s_gc_max[i] = g[i];
    }
    if (++s_gc_n < GYROCAL_N) return;

    float b[3];
    imu_get_gyro_bias(&b[0], &b[1], &b[2]);
    s_gc_moved = false;
    for (int i = 0; i < 3; i++) {
        s_gc_bias[i] = b[i] + (float)(s_gc_sum[i] / s_gc_n);
        if (s_gc_max[i] - s_gc_min[i] > GYROCAL_MOVE_DPS) s_gc_moved = true;
    }
    if (!s_gc_moved) {
        imu_set_gyro_bias(s_gc_bias[0], s_gc_bias[1], s_gc_bias[2]);
        estimator_reset();     /* re-seed tilt to converge with the debiased gyro */
    }
    s_gc_active = false;
    s_gc_report = true;
}

/* ISR context: keep it minimal - just wake the control task for the next tick. */
static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx)
{
    BaseType_t high_prio_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_control_task, &high_prio_woken);
    return high_prio_woken == pdTRUE;
}

/* Create + start the GPTimer once, stashing the handle so a restart just re-starts it. */
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

/* One tick of the deadband sweep: ramp the active direction and latch, per wheel,
 * the duty where the encoder first moves (freezing a tripped wheel at 0). Advances
 * forward -> settle -> reverse -> done, publishing s_db_report at the end. */
static void deadband_step(int64_t now_us, float *cmdL, float *cmdR)
{
    float elapsed = (float)(now_us - s_db_t0_us) * 1e-6f;
    *cmdL = 0.0f;
    *cmdR = 0.0f;

    if (s_db_phase == DB_PHASE_SETTLE) {   /* park, then re-baseline for reverse */
        if (elapsed >= DB_SETTLE_S) {
            s_db_phase = DB_PHASE_REV;
            s_db_t0_us = now_us;
            s_db_start_cnt[0] = encoder_count(0);
            s_db_start_cnt[1] = encoder_count(1);
        }
        return;
    }

    int   dir = (s_db_phase == DB_PHASE_FWD) ? 0 : 1;
    float sgn = (s_db_phase == DB_PHASE_FWD) ? 1.0f : -1.0f;
    float duty = DB_RAMP_RATE * elapsed;
    if (duty > DB_MAX) duty = DB_MAX;

    float cmd[2] = { 0.0f, 0.0f };
    for (int i = 0; i < 2; i++) {
        if (!s_db_found[i][dir]) {
            int64_t moved = encoder_count(i) - s_db_start_cnt[i];
            if (moved < 0) moved = -moved;
            if (moved >= DB_MOVE_COUNTS) {
                s_db_thresh[i][dir] = duty;
                s_db_found[i][dir]  = true;
            }
        }
        cmd[i] = s_db_found[i][dir] ? 0.0f : sgn * duty;
    }

    bool done = (s_db_found[0][dir] && s_db_found[1][dir]) || duty >= DB_MAX;
    if (done) {
        for (int i = 0; i < 2; i++) {
            if (!s_db_found[i][dir]) s_db_thresh[i][dir] = DB_MAX;   /* never moved */
        }
        cmd[0] = 0.0f;
        cmd[1] = 0.0f;
        if (s_db_phase == DB_PHASE_FWD) {
            s_db_phase = DB_PHASE_SETTLE;
            s_db_t0_us = now_us;
        } else {
            s_db_active = false;    /* reverse done: park and hand off to reporter */
            s_db_report = true;
        }
    }
    *cmdL = cmd[0];
    *cmdR = cmd[1];
}

/* High-priority task woken by the timer ISR every 1/CONTROL_HZ s. Records one telemetry
 * sample every REPORT_DIV ticks; swaps buffers every SAMPLES_PER_BATCH and notifies the reporter. */
static void control_task(void *arg)
{
    /* Claim this task's handle for the ISR before the timer can fire. */
    s_control_task = xTaskGetCurrentTaskHandle();

    /* First start only: bring up peripherals from THIS task so their interrupts land
     * on core 1 (esp_intr_alloc pins to the calling core), insulating the real-time
     * path from core 0's Wi-Fi. A restart reuses them, so skip re-init. */
    if (!s_periph_ready) {
        encoder_init();   /* PCNT quadrature counters, count from 0 */
        motor_init();     /* LEDC PWM channels; motors start stopped */
        i2c_init();       // I2C master bus, created once and reused on restart
        i2c_scan();
        s_periph_ready = true;
    }

    /* Retry the IMU on every (re)start until it succeeds (init is idempotent), so
     * fixing the wiring and restarting the control task brings it up, no reboot. */
    if (!s_imu_present) {
        s_imu_present = mpu6050_init();
        if (!s_imu_present) {
            ESP_LOGW(TAG, "no IMU - balance disabled, skipping IMU reads; motors + "
                          "encoders still run (check wiring at SDA=21 SCL=22, then "
                          "restart the control task to retry)");
        }
    }

    /* Start the GPTimer from THIS task so its interrupt is pinned to core 1. First
     * start creates it; a restart re-starts the existing timer. */
    if (s_timer == NULL) {
        start_control_timer();
    } else {
        ESP_ERROR_CHECK(gptimer_start(s_timer));
    }

    imu_t imu = { 0 };             /* latest IMU sample */
    /* Ring buffer of the last VEL_WIN ticks' counts + timestamps (windowed speed). */
    int64_t vel_hist_t[VEL_WIN] = { 0 };
    int64_t vel_hist[2][VEL_WIN] = { { 0 } };
    int     vel_head = 0;          /* next slot to write */
    int     vel_n = 0;             /* samples buffered so far (<= VEL_WIN) */
    int64_t last_us = esp_timer_get_time();   /* timestamp of the previous tick */
    bool    primed = false;        /* skip the warning check on the first tick */
    bool    ctrl_was = false;      /* previous tick's controller-enabled state */
    bool    bal_was  = false;      /* previous tick's balance-enabled state */
    uint32_t slow_count = 0;       // ticks since the last sub-rate (BALANCE_HZ) fire
    int64_t  slow_dt_us = 0;       // time accumulated since that fire [us]
    bool     bal_cut   = false;    // latched: motors cut because tilt exceeded the limit
    uint32_t imu_fail_streak = 0;  // consecutive failed reads; drops the IMU past the limit
    float    roll = NAN, pitch = NAN;  // latest estimate [rad], held between sub-rate fires
    uint32_t rpt_count = 0;        // ticks since the last recorded telemetry sample
    int     active = 0;            /* buffer we are currently writing into */
    int     idx = 0;               /* next slot in the active buffer */

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* STOP_CONTROL requested: break at a safe point (never mid-I2C) for cleanup below. */
        if (s_stop_req) break;

        int64_t now_us = esp_timer_get_time();
        int32_t dt_us  = (int32_t)(now_us - last_us);
        last_us = now_us;

        /* Check this tick's period; out-of-range only stashes a value for the reporter. */
        if (primed && (dt_us > DT_MAX_WARN_US || dt_us < DT_MIN_WARN_US)) {
            s_warn_dt_us = dt_us;
            s_warn_count++;
        }
        primed = true;

        /* Sub-rate gate (BALANCE_HZ) for the IMU read, estimator and balance; slow_dt is
         * their true accumulated period. Wheel loop and telemetry stay at CONTROL_HZ. */
        slow_dt_us += dt_us;
        bool  slow_tick = (++slow_count >= BALANCE_DIV);
        float slow_dt   = 0.0f;
        if (slow_tick) {
            slow_count = 0;
            slow_dt    = (slow_dt_us > 0) ? (float)slow_dt_us * 1e-6f
                                          : (float)BALANCE_DIV / (float)CONTROL_HZ;
            slow_dt_us = 0;
        }

        /* Read the IMU on sub-rate ticks (~0.5 ms blocking I2C). On failure/timeout hold
         * the last good sample and flag it stale; with no IMU we never touch the bus. */
        if (slow_tick && s_imu_present) {
            int64_t imu_t0    = esp_timer_get_time();
            imu_t   imu_fresh = mpu6050_read();
            int32_t imu_us    = (int32_t)(esp_timer_get_time() - imu_t0);
            if (imu_us > s_warn_imu_us) s_warn_imu_us = imu_us;
            if (imu_fresh.ok) {
                imu = imu_fresh;       /* fresh sample */
                imu_fail_streak = 0;
            } else {
                imu.ok = false;        /* keep last good ax..gz, flag stale (estimator holds) */
                s_imu_stale_count++;
                /* A long run of failures means the IMU is gone (unplugged/dead). Drop it so
                 * we stop hammering the bus and balance disables; a control restart re-inits.
                 * The reporter task surfaces the event to the web terminal (off the RT path). */
                if (++imu_fail_streak > IMU_FAIL_LIMIT) {
                    s_imu_present  = false;
                    s_imu_lost_evt = true;
                    imu_fail_streak = 0;
                }
            }
        } else if (slow_tick) {
            imu.ok = false;            /* no IMU: no read, no estimate, no balance */
        }

        // Gyro-bias calibration when armed; before the estimator so it re-seeds same tick.
        if (slow_tick && s_gc_active) gyrocal_step(imu);

        /* Complementary-filter tilt estimate, held between sub-rate fires. NaN when
         * estimation is off, no IMU, or not yet seeded. */
        if (slow_tick) {
            if (!s_est_enabled || !s_imu_present) {
                estimator_reset();
                roll = NAN; pitch = NAN;
            } else if (!estimator_update(imu, slow_dt, &roll, &pitch)) {
                roll = NAN; pitch = NAN;   /* not seeded yet */
            }
        }

        /* Wheel angular speed (rad/s), before the command decision so the controller
         * can use it this tick. Finite difference over a VEL_WIN-sample window using
         * actual elapsed time (jitter-safe); averaging cuts encoder quantization noise. */
        int64_t posL_cnt = encoder_count(0);
        int64_t posR_cnt = encoder_count(1);
        vel_hist_t[vel_head]  = now_us;
        vel_hist[0][vel_head] = posL_cnt;
        vel_hist[1][vel_head] = posR_cnt;
        int vel_new = vel_head;
        if (vel_n < VEL_WIN) vel_n++;
        vel_head = (vel_head + 1) % VEL_WIN;
        int vel_old = (vel_new - (vel_n - 1) + VEL_WIN) % VEL_WIN;
        float velL = 0.0f, velR = 0.0f;
        int64_t win_us = vel_hist_t[vel_new] - vel_hist_t[vel_old];
        if (win_us > 0) {
            float scale = 1e6f / (float)win_us;   /* counts over the window -> counts/sec */
            velL = encoder_cps_to_radps((int32_t)((vel_hist[0][vel_new] - vel_hist[0][vel_old]) * scale));
            velR = encoder_cps_to_radps((int32_t)((vel_hist[1][vel_new] - vel_hist[1][vel_old]) * scale));
        }

        /* Motor commands. Closed loop: from the wheel controller. Open loop: a loaded
         * playback script takes priority, else the effort from the web terminal.
         * ('stop' only zeroes the open-loop command; STOP_CONTROL deletes the task.) */
        float cmdL, cmdR;
        float wsetL = NAN, wsetR = NAN;   /* setpoints for telemetry (NaN when open loop) */
        float rawL = NAN, rawR = NAN;         /* raw PI output pre-deadband/sat (NaN when open loop) */
        if (s_ctrl_enabled) {
            /* Rising edge into closed loop: clear integrators/filters for a clean start. */
            if (!ctrl_was) wheel_pi_reset();
            float dt = (dt_us > 0) ? (float)dt_us * 1e-6f : (1.0f / (float)CONTROL_HZ);

            /* Balance (outer) loop: turn the estimated tilt into a common wheel-speed
             * setpoint both inner loops track. If tilt exceeds BALANCE_MAX_TILT the
             * catch strategy can't recover, so cut the motors rather than lunge. */
            bool balance_cut = false;
            if (s_bal_enabled) {
                /* Rising edge: clear state. */
                if (!bal_was) {
                    balance_pid_reset();
                    bal_cut = false;
                }
                /* Fires on the sub-rate tick (fresh estimate, slow_dt period so Ki stays
                 * correct); setpoints held (ZOH) between fires, inner loops track every tick. */
                if (slow_tick) {
                    bool have_theta = s_imu_present && s_est_enabled && isfinite(pitch);
                    if (have_theta && fabsf(pitch) < BALANCE_MAX_TILT) {
                        float theta_dot = imu.gy * DEG2RAD;   /* tilt rate about Y, rad/s */
                        float w_common  = balance_pid_step(pitch, theta_dot, slow_dt);
                        wheel_pi_set_setpoint(0, w_common);
                        wheel_pi_set_setpoint(1, w_common);
                        bal_cut = false;
                    } else {
                        balance_pid_reset();
                        wheel_pi_reset();
                        wheel_pi_set_setpoint(0, 0.0f);
                        wheel_pi_set_setpoint(1, 0.0f);
                        bal_cut = true;
                    }
                }
                /* Hold the last cut decision between updates so a fallen robot stays cut. */
                balance_cut = bal_cut;
            }

            if (balance_cut) {
                cmdL = 0.0f;
                cmdR = 0.0f;
            } else {
                /* Per-wheel PI + deadband + anti-windup -> duty; dt is the measured
                 * period so gains stay correct under jitter. Setpoints come from the
                 * outer loop (balancing) or the 'speed' command. */
                wsetL = wheel_pi_setpoint(0);
                wsetR = wheel_pi_setpoint(1);
                cmdL = wheel_pi_step(0, velL, dt);
                cmdR = wheel_pi_step(1, velR, dt);
                rawL = wheel_pi_raw(0);
                rawR = wheel_pi_raw(1);
            }
        } else if (s_db_active) {
            /* Deadband sweep takes priority over playback/manual while running. */
            deadband_step(now_us, &cmdL, &cmdR);
        } else if (s_play_active) {
            /* Time-based playback: skip finished steps, apply the current one, park at the end. */
            float elapsed = (float)(now_us - s_play_t0_us) * 1e-6f;
            int i = s_play_idx;
            while (i < s_play_len && elapsed >= s_play[i].t_end) i++;
            s_play_idx = i;
            if (i >= s_play_len) {
                s_play_active = false;      /* end of script: park + open loop */
                s_play_report = true;       /* reporter announces completion (core 0) */
                motor_cmd_set(0, 0.0f);
                motor_cmd_set(1, 0.0f);
                cmdL = 0.0f;
                cmdR = 0.0f;
            } else {
                cmdL = s_play[i].mL;
                cmdR = s_play[i].mR;
            }
        } else {
            cmdL = motor_cmd_get(0);
            cmdR = motor_cmd_get(1);
        }
        motor_set(0, cmdL);
        motor_set(1, cmdR);
        ctrl_was = s_ctrl_enabled;
        bal_was  = s_bal_enabled;

        /* Record telemetry every REPORT_DIV-th tick (REPORT_HZ) to cut streamed data;
         * control still runs full rate. Wheel angle is the continuous count; speeds reuse
         * the values above. wsetL/wsetR are the closed-loop setpoints (NaN when open). */
        if (++rpt_count >= REPORT_DIV) {
            rpt_count = 0;
            sample_t *smp = &s_buf[active][idx];
            smp->t_us = now_us;
            smp->posL = (float)encoder_angle_rad(0);
            smp->posR = (float)encoder_angle_rad(1);
            smp->velL = velL;
            smp->velR = velR;
            smp->roll  = roll;
            smp->pitch = pitch;
            smp->wsetL = wsetL;
            smp->wsetR = wsetR;
            smp->rawL   = rawL;
            smp->rawR   = rawR;
            smp->mL   = cmdL;
            smp->mR   = cmdR;
            smp->imu  = imu;

            /* Buffer full: hand it to the reporter (timeout 0, never blocks) and swap
             * halves. If the reporter fell behind we drop this batch, never stall. */
            if (++idx >= SAMPLES_PER_BATCH) {
                if (xQueueSend(s_ready_q, &active, 0) != pdPASS) {
                    s_drop_count++;   /* reporter behind; the reporter logs it, not us */
                }
                active = !active;
                idx = 0;
            }
        }

        /* Compute-budget watchdog: time this tick's work; keep the worst for the
         * reporter. Over 90% of the period means we're one hiccup from missing a tick. */
        int32_t run_us = (int32_t)(esp_timer_get_time() - now_us);
        if (run_us > RUN_WARN_US) {
            if (run_us > s_warn_run_us) s_warn_run_us = run_us;
            s_warn_run_count++;
        }
    }

    /* STOP_CONTROL only: stop the timer so the ISR can't notify a deleted task,
     * park the motors, then self-delete. A restart creates a fresh task on the same timer. */
    ESP_ERROR_CHECK(gptimer_stop(s_timer));
    motor_set(0, 0.0f);
    motor_set(1, 0.0f);
    s_control_task = NULL;
    s_stop_req     = false;
    s_mode         = STOP_CONTROL;
    vTaskDelete(NULL);
}

/* Announce any timing violations since the last batch to the console and web
 * terminal. Coalesces a burst into one message with a count. */
static void emit_pending_warning(void)
{
    char msg[192];
    char json[288];

    /* Period out of range (tick scheduled early/late). */
    uint32_t n = s_warn_count;
    if (n != 0) {
        int32_t dt = s_warn_dt_us;
        s_warn_count = 0;        /* clear before reporting; races are harmless */
        snprintf(msg, sizeof(msg),
                 "loop timing out of range: %" PRIu32 " tick(s) in the last %d ms, "
                 "latest dt=%ld us (want ~%d, limits %d..%d)",
                 n, BATCH_MS, (long)dt,
                 CONTROL_PERIOD_US, DT_MIN_WARN_US, DT_MAX_WARN_US);
        ESP_LOGW(TAG, "%s", msg);
        snprintf(json, sizeof(json), "{\"type\":\"warn\",\"text\":\"%s\"}", msg);
        ws_broadcast(json);
    }

    /* Compute budget exceeded. Include the per-phase split (worst IMU read is the
     * only blocking call) so the message says whether it's the I2C read or the compute. */
    uint32_t rn = s_warn_run_count;
    if (rn != 0) {
        int32_t run = s_warn_run_us;
        s_warn_run_count = 0;
        s_warn_run_us    = 0;    /* reset the worst-case tracker for next window */
        snprintf(msg, sizeof(msg),
                 "control task over budget: %" PRIu32 " tick(s) in the last %d ms, "
                 "worst run=%ld us (limit %d us = 90%% of %d); worst IMU read=%ld us",
                 rn, BATCH_MS, (long)run,
                 RUN_WARN_US, CONTROL_PERIOD_US, (long)s_warn_imu_us);
        ESP_LOGW(TAG, "%s", msg);
        snprintf(json, sizeof(json), "{\"type\":\"warn\",\"text\":\"%s\"}", msg);
        ws_broadcast(json);
    }

    /* Stale IMU reads (I2C timed out; held last sample). Reported on its own since
     * the fast-failing read no longer trips the budget warning. Reset the IMU trackers here. */
    uint32_t stale = s_imu_stale_count;
    int32_t  imu_worst = s_warn_imu_us;
    s_imu_stale_count = 0;
    s_warn_imu_us     = 0;
    if (stale != 0) {
        snprintf(msg, sizeof(msg),
                 "IMU read stalled: %" PRIu32 " stale read(s) in the last %d ms "
                 "(held last sample); worst IMU read=%ld us",
                 stale, BATCH_MS, (long)imu_worst);
        ESP_LOGW(TAG, "%s", msg);
        snprintf(json, sizeof(json), "{\"type\":\"warn\",\"text\":\"%s\"}", msg);
        ws_broadcast(json);
    }

    /* Telemetry batches dropped because this reporter couldn't keep up. */
    uint32_t drops = s_drop_count;
    if (drops != 0) {
        s_drop_count = 0;
        snprintf(msg, sizeof(msg),
                 "reporter behind: dropped %" PRIu32 " telemetry batch(es) in the last %d ms",
                 drops, BATCH_MS);
        ESP_LOGW(TAG, "%s", msg);
        snprintf(json, sizeof(json), "{\"type\":\"warn\",\"text\":\"%s\"}", msg);
        ws_broadcast(json);
    }

    /* IMU dropped after a long failure run: balance is now disabled until a restart. */
    if (s_imu_lost_evt) {
        s_imu_lost_evt = false;
        snprintf(msg, sizeof(msg),
                 "IMU lost after %d consecutive failed reads - balance disabled; "
                 "restart the control task to retry",
                 IMU_FAIL_LIMIT);
        ESP_LOGW(TAG, "%s", msg);
        snprintf(json, sizeof(json), "{\"type\":\"error\",\"text\":\"%s\"}", msg);
        ws_broadcast(json);
    }
}

/* Announce the deadband sweep's four thresholds once the loop finishes, off the
 * RT path. A wheel that never moved before DB_MAX is reported as ">=DB_MAX". */
static void emit_deadband_result(void)
{
    if (!s_db_report) return;
    s_db_report = false;

    /* Column order L+, L-, R+, R-  ->  [wheel][0=fwd,1=rev]. */
    const int wi[4] = { 0, 0, 1, 1 };
    const int di[4] = { 0, 1, 0, 1 };
    char cell[4][12];
    for (int k = 0; k < 4; k++) {
        if (s_db_found[wi[k]][di[k]]) {
            snprintf(cell[k], sizeof(cell[k]), "%.3f", (double)s_db_thresh[wi[k]][di[k]]);
        } else {
            snprintf(cell[k], sizeof(cell[k]), ">=%.2f", (double)DB_MAX);
        }
    }
    char msg[160];
    snprintf(msg, sizeof(msg), "deadband (duty): L +%s / -%s   R +%s / -%s",
             cell[0], cell[1], cell[2], cell[3]);
    ESP_LOGI(TAG, "%s", msg);
    char json[256];
    snprintf(json, sizeof(json), "{\"type\":\"resp\",\"text\":\"%s\"}", msg);
    ws_broadcast(json);
}

/* Announce that a playback script ran to completion, once, off the RT path.
 * Manual 'play stop' has its own reply and does not set this. */
static void emit_playback_result(void)
{
    if (!s_play_report) return;
    s_play_report = false;

    float total = (s_play_len > 0) ? s_play[s_play_len - 1].t_end : 0.0f;
    char msg[96];
    snprintf(msg, sizeof(msg), "playback finished (%d steps, %.2f s), motors parked",
             s_play_len, (double)total);
    ESP_LOGI(TAG, "%s", msg);
    char json[160];
    snprintf(json, sizeof(json), "{\"type\":\"resp\",\"text\":\"%s\"}", msg);
    ws_broadcast(json);
}

/* Announce the gyro-bias calibration result once, off the RT path. On motion the
 * bias is left unchanged and the operator is told to retry holding the robot still. */
static void emit_gyrocal_result(void)
{
    if (!s_gc_report) return;
    s_gc_report = false;

    char msg[192];
    if (s_gc_moved) {
        snprintf(msg, sizeof(msg),
                 "gyro calib aborted: motion detected (spread > %.1f dps) - "
                 "hold the robot still and retry", (double)GYROCAL_MOVE_DPS);
    } else {
        snprintf(msg, sizeof(msg),
                 "gyro bias set: gx=%.3f gy=%.3f gz=%.3f dps (avg of %d samples)",
                 (double)s_gc_bias[0], (double)s_gc_bias[1], (double)s_gc_bias[2],
                 GYROCAL_N);
    }
    ESP_LOGI(TAG, "%s", msg);
    char json[256];
    snprintf(json, sizeof(json), "{\"type\":\"resp\",\"text\":\"%s\"}", msg);
    ws_broadcast(json);
}

/* Runs on core 0. Blocks until the control task hands over a full buffer, then emits
 * pending warnings, caches the latest sample, and streams the batch to clients. */
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

        /* Announce the deadband sweep result when the loop has finished one. */
        emit_deadband_result();

        /* Announce when a playback script has run to completion. */
        emit_playback_result();

        /* Announce the gyro-bias calibration result when the loop finishes one. */
        emit_gyrocal_result();

        /* Pack one binary frame per topic and stream it to WebSocket clients. The
         * scratch buffer is static (too big for the task stack, and avoids the heap
         * fragmentation a per-batch malloc caused). Only reporter_task touches it. */
        if (web_server_streaming()) {
            static uint8_t frame[REPORT_FRAME_CAP];
            static uint32_t batch_no = 0;
            for (int t = 0; t < telemetry_topic_count(); t++) {
                /* Throttle the bulky imu topic to ~1 Hz; stream the rest every batch. */
                if (strcmp(telemetry_topic_name(t), "imu") == 0 &&
                    (batch_no % IMU_STREAM_DIV) != 0) {
                    continue;
                }
                int n = telemetry_topic_pack(frame, sizeof(frame), t, batch,
                                             SAMPLES_PER_BATCH);
                if (n > 0) {
                    ws_broadcast_bin(frame, (size_t)n);
                } else {
                    ESP_LOGW(TAG, "topic '%s' frame did not fit in %u bytes",
                             telemetry_topic_name(t), (unsigned)sizeof(frame));
                }
            }
            batch_no++;
        }
    }
}

void control_start(void)
{
    /* Ready-buffer index queue. Length 2 so the control task can hand over even
     * while the reporter is mid-cycle on the previous buffer. */
    s_ready_q = xQueueCreate(2, sizeof(int));
    configASSERT(s_ready_q);

    /* Reporter on core 0 (shares with Wi-Fi). If Wi-Fi/WebSocket stalls, only this
     * task waits; the control loop is unaffected. */
    xTaskCreatePinnedToCore(reporter_task, "reporter", 4096, NULL, 4, NULL, 0);

    /* Start the control task (created on core 1, brings up peripherals + GPTimer there). */
    control_set_mode(START_CONTROL);
}

control_mode_t control_mode(void) { return s_mode; }

void control_set_mode(control_mode_t mode)
{
    if (mode == START_CONTROL) {
        if (s_mode == START_CONTROL) return;          /* already running */
        s_stop_req = false;
        s_mode     = START_CONTROL;
        /* Control loop on core 1, high priority, isolated from core 0's Wi-Fi. */
        xTaskCreatePinnedToCore(control_task, "control", 4096, NULL,
                                configMAX_PRIORITIES - 2, NULL, 1);
    } else {  /* STOP_CONTROL */
        if (s_mode == STOP_CONTROL) return;           /* already stopped */
        /* Ask the loop to exit at a safe point (stops timer, parks motors, self-deletes),
         * then wait (bounded) until it's gone so callers know it stopped on return. */
        s_stop_req = true;
        for (int i = 0; i < 200 && s_control_task != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        s_mode = STOP_CONTROL;
    }
}

void control_set_experiment(experiment_mode_t mode)
{
    switch (mode) {
        case TEST_MOTOR_CONTROLLERS: s_est_enabled = false; s_ctrl_enabled = true;  s_bal_enabled = false; break;
        case TEST_ANGLES_ESTIMATION: s_est_enabled = true;  s_ctrl_enabled = false; s_bal_enabled = false; break;
        case TEST_BALANCE:           s_est_enabled = true;  s_ctrl_enabled = true;  s_bal_enabled = true;  break;
        case TEST_MOTORS:
        default:                     s_est_enabled = false; s_ctrl_enabled = false; s_bal_enabled = false; break;
    }
}

bool control_estimation_enabled(void) { return s_est_enabled; }
void control_set_estimation(bool on)  { s_est_enabled = on; }
bool control_controller_enabled(void) { return s_ctrl_enabled; }
void control_set_controller(bool on)  { s_ctrl_enabled = on; }
bool control_balance_enabled(void)    { return s_bal_enabled; }
void control_set_balance(bool on)     { s_bal_enabled = on; }

void control_playback_begin(void)
{
    s_play_active = false;   /* stop the loop touching it before we refill */
    s_play_idx = 0;
    s_play_len = 0;
}

int control_playback_append(float dur, float mL, float mR)
{
    if (s_play_len >= PLAYBACK_MAX) return -1;
    if (dur <= 0.0f) return -2;             /* a step must last a positive time */
    if (mL >  1.0f) mL =  1.0f; else if (mL < -1.0f) mL = -1.0f;
    if (mR >  1.0f) mR =  1.0f; else if (mR < -1.0f) mR = -1.0f;
    int i = s_play_len;
    float prev_end = (i > 0) ? s_play[i - 1].t_end : 0.0f;
    s_play[i].t_end = prev_end + dur;       /* durations -> cumulative end time */
    s_play[i].mL    = mL;
    s_play[i].mR    = mR;
    s_play_len = i + 1;      /* publish last so the loop only sees full entries */
    return s_play_len;
}

void control_playback_start(void)
{
    if (s_play_len > 0) {
        s_play_idx    = 0;
        s_play_report = false;                  /* clear any stale completion event */
        s_play_t0_us  = esp_timer_get_time();   /* clock starts at 0 */
        s_play_active = true;
    }
}

void control_playback_stop(void)
{
    s_play_active = false;
    motor_cmd_set(0, 0.0f);
    motor_cmd_set(1, 0.0f);
}

bool control_playback_active(void) { return s_play_active; }
int  control_playback_len(void)    { return s_play_len; }
int  control_playback_pos(void)    { return s_play_idx; }

void control_deadband_start(void)
{
    s_play_active = false;             /* don't let playback fight the sweep */
    for (int i = 0; i < 2; i++) {
        for (int d = 0; d < 2; d++) { s_db_thresh[i][d] = 0.0f; s_db_found[i][d] = false; }
    }
    s_db_phase   = DB_PHASE_FWD;
    s_db_t0_us   = esp_timer_get_time();
    s_db_start_cnt[0] = encoder_count(0);
    s_db_start_cnt[1] = encoder_count(1);
    s_db_report  = false;
    s_db_active  = true;               /* publish last so the loop only sees a ready state */
}

void control_deadband_stop(void)
{
    s_db_active = false;
    motor_cmd_set(0, 0.0f);
    motor_cmd_set(1, 0.0f);
}

bool control_deadband_active(void) { return s_db_active; }

void control_gyrocal_start(void)
{
    s_gc_n      = 0;
    s_gc_moved  = false;
    s_gc_report = false;
    s_gc_active = true;                /* publish last so the loop sees a ready state */
}

bool control_gyrocal_active(void) { return s_gc_active; }
