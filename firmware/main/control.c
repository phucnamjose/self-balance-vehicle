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
#define ALARM_COUNT    (TIMER_RES_HZ / CONTROL_HZ)       /* 500 Hz motor tick */

/* Wheel speed = finite difference of encoder counts over a sliding VEL_WIN-tick
 * window; averaging cuts count-quantization noise (~10 ms window at 500 Hz).
 * A longer window trades resolution for delay - wrong on a latency-limited loop. */
#define VEL_WIN        5

/* Gyro is reported in deg/s; the balance loop wants the tilt rate in rad/s. */
#define DEG2RAD        (float)(M_PI / 180.0)

/* The IMU read and orientation estimate run in a separate imu_task at this rate -
 * a healthy margin above the ~32 rad/s crossover. Running the costly, blocking
 * MPU6050 read in its own lower-priority task keeps it off the motor tick, so an
 * I2C stall can never blow the 500 Hz budget. The MPU samples at 500 Hz into its
 * FIFO; the task consumes the newest at IMU_HZ. */
#define IMU_HZ          250
#define IMU_ALARM_COUNT (TIMER_RES_HZ / IMU_HZ)          /* 250 Hz IMU + estimator tick */

/* Balance (outer) PID rate: it runs every BALANCE_DIV-th imu_task tick, i.e. at
 * BALANCE_HZ. The estimator always fuses at the full IMU_HZ. BALANCE_HZ == IMU_HZ
 * (BALANCE_DIV = 1) runs the PID every tick (least ZOH lag); drop BALANCE_HZ to
 * sub-sample it, keeping it an integer divisor of IMU_HZ. */
#define BALANCE_HZ      250
#define BALANCE_DIV     (IMU_HZ / BALANCE_HZ)            /* imu ticks per balance step (= 1) */

/* Consecutive stale IMU ticks that mean it is gone (~400 ms at 250 Hz), not a blip. */
#define IMU_FAIL_LIMIT 100

/* Emit the periodic loop-stats line every STATS_DIV telemetry batches (~1 s). */
#define STATS_DIV      (1000 / BATCH_MS)
#define STATS_MS       (STATS_DIV * BATCH_MS)

/* Scratch buffer for one packed telemetry frame, derived from SAMPLES_PER_BATCH so
 * it tracks CONTROL_HZ (a fixed size overflowed at 500 Hz and dropped topics).
 * Keep REPORT_FRAME_MAX_FIELDS >= the widest topic's field count. +64 B slack. */
#define REPORT_FRAME_MAX_FIELDS 9
#define REPORT_FRAME_CAP  (4 + SAMPLES_PER_BATCH * (8 + (REPORT_FRAME_MAX_FIELDS - 1) * 4) + 64)

/* Stream one newest imu sample per second: batches arrive at REPORT_HZ/SAMPLES_PER_BATCH
 * Hz, so every IMU_STREAM_DIV-th batch is one per second. */
#define IMU_STREAM_DIV  (REPORT_HZ / SAMPLES_PER_BATCH)

/* Two RT tasks on core 1: motor_task (500 Hz, hard-RT, no I2C) and imu_task (250 Hz,
 * one priority below, owns the blocking IMU read + estimator + balance). */
static TaskHandle_t s_motor_task;
static TaskHandle_t s_imu_task;

/* imu_task -> motor_task control handoff (single 32-bit values, atomic on Xtensa). The
 * balance loop publishes the common wheel-speed setpoint and a fall-cut flag; the motor
 * loop tracks them via ZOH every tick. */
static volatile float s_bal_w_common;   /* balance output, rad/s */
static volatile bool  s_bal_cut;         /* tilt exceeded: motor loop zeroes the wheels */

/* imu_task -> motor_task telemetry snapshot (imu + estimate), published under a seqlock
 * so the motor loop reads a consistent frame without a mutex on the RT path. */
static volatile uint32_t s_pub_seq;      /* even = stable, odd = write in progress */
static imu_t s_pub_imu;
static float s_pub_roll = NAN, s_pub_pitch = NAN;

/* Double buffer: motor_task fills one half while the reporter drains the other,
 * swapping every SAMPLES_PER_BATCH. The queue passes the filled buffer's index. */
static sample_t      s_buf[2][SAMPLES_PER_BATCH];
static QueueHandle_t s_ready_q;

/* Publish the estimate + IMU snapshot (imu_task). Bracket the write with an odd->even
 * seq bump; the barriers stop the compiler reordering the payload past the seq. */
static void imu_pub_write(const imu_t *imu, float roll, float pitch)
{
    s_pub_seq++;
    __asm__ volatile("" ::: "memory");
    s_pub_imu = *imu; s_pub_roll = roll; s_pub_pitch = pitch;
    __asm__ volatile("" ::: "memory");
    s_pub_seq++;
}

/* Read the snapshot (motor_task). One attempt; if the writer is mid-update, leave the
 * outputs untouched (the caller keeps its previous copy) - never spin, since the writer
 * is lower priority and could not make progress while we wait. */
static void imu_pub_try_read(imu_t *imu, float *roll, float *pitch)
{
    uint32_t s0 = s_pub_seq;
    if (s0 & 1u) return;
    __asm__ volatile("" ::: "memory");
    imu_t  i = s_pub_imu;
    float  r = s_pub_roll, p = s_pub_pitch;
    __asm__ volatile("" ::: "memory");
    if (s_pub_seq != s0) return;
    *imu = i; *roll = r; *pitch = p;
}

/* Timing-warning handoff: the loop records an out-of-range period and bumps the
 * counter; the reporter (off the RT path) drains it into one warning per batch. */
static volatile int32_t  s_warn_dt_us;   /* most recent out-of-range period */
static volatile uint32_t s_warn_count;   /* violations since the last report */

/* Same handoff for per-tick run time: stash the worst over-budget value and count
 * it. Separate from the period warning so slow-to-schedule vs slow-to-compute is clear. */
static volatile int32_t  s_warn_run_us;    /* worst over-budget run time */
static volatile uint32_t s_warn_run_count; /* overruns since the last report */

/* Missed-tick metrics: when a task is late, ulTaskNotifyTake returns >1 pending ticks;
 * we run the latest once and count the rest here (drained by the reporter each window). */
static volatile uint32_t s_motor_missed;   /* 500 Hz ticks the motor task ran late for */
static volatile uint32_t s_imu_missed;     /* 250 Hz ticks the IMU task ran late for */
static volatile uint32_t s_imu_skipped;    /* older FIFO frames dropped (take-newest) */

/* Telemetry batches dropped because the reporter fell behind. Counted on the RT
 * path (never logged there), drained by the reporter. */
static volatile uint32_t s_drop_count;

/* Per-phase profiling: the IMU read is the tick's only variable-latency blocking
 * call (I2C), so time it separately. s_imu_stale_count counts failed reads. */
static volatile int32_t  s_warn_imu_us;      /* worst IMU-read time in the window */
static volatile uint32_t s_imu_stale_count;  /* failed/timed-out reads (held last) */
static volatile bool     s_imu_lost_evt;     /* set when the IMU is dropped; reporter broadcasts it */

/* Loop-stats accumulators: each task sums its per-tick run time (+ the IMU task sums
 * new samples/tick), the reporter averages + resets them once per STATS window. These
 * averages let you sanity-check an occasional out-of-range period against the typical
 * run time, which normally sits well under budget. Sums fit uint32 over the window. */
static volatile uint32_t s_motor_run_sum;    /* sum of motor tick run times [us] */
static volatile uint32_t s_motor_run_cnt;    /* motor ticks summed */
static volatile uint32_t s_imu_run_sum;      /* sum of IMU tick run times [us] */
static volatile uint32_t s_imu_run_cnt;      /* IMU ticks summed */
static volatile uint32_t s_imu_samp_sum;     /* sum of new samples seen per IMU tick */

/* Last-window averages, published by the reporter for the stats command. */
static volatile float s_stat_motor_run_us;   /* mean motor tick run time [us] */
static volatile float s_stat_imu_run_us;     /* mean IMU tick run time [us] */
static volatile float s_stat_imu_samples;    /* mean new IMU samples per IMU tick */

/* Control lifecycle: STOP_CONTROL deletes both tasks and parks the motors;
 * START_CONTROL re-creates them. The GPTimers/peripherals are set up once and reused. */
static volatile control_mode_t s_mode = STOP_CONTROL;  /* set to START by control_start() */
static gptimer_handle_t  s_timer;        /* 500 Hz motor timer, created once, kept across stop/start */
static gptimer_handle_t  s_imu_timer;    /* 250 Hz IMU timer, likewise */
static bool              s_periph_ready; /* peripherals init'd once, on first start */
static volatile bool     s_imu_present;  /* mpu6050_init() succeeded (motor_task sets it, imu_task
                                          * clears it on loss); if false, imu_task skips the bus
                                          * and balance stays disabled */
static volatile bool     s_stop_req;     /* ask both loops to exit at a safe point */

/* Research feature flags, read every tick. Default (all off) is a plain open-loop
 * motor test - no estimation or controller. */
static volatile bool     s_est_enabled;  /* run the orientation (roll/pitch) estimate */
static volatile bool     s_ctrl_enabled; /* run the wheel/motor controller */
static volatile bool     s_bal_enabled;  /* run the balance (tilt) outer loop */

/* Scripted playback: hold (vL,vR) per step for a duration - duty (TEST_MOTORS) or
 * wheel-speed setpoints (TEST_MOTOR_CONTROLLERS) per s_play_kind. Durations are stored
 * as cumulative end times. Static buffer so the loop never touches freed memory; the
 * uploader publishes s_play_len last. */
typedef struct { float t_end, vL, vR; } play_pt_t;   /* t_end: step end, s from start */
#define PLAYBACK_MAX   128       /* step-array capacity (fixed for RT safety) */
#define PLAYBACK_MAX_S 60.0f     /* total script duration cap [s] */
static play_pt_t         s_play[PLAYBACK_MAX];
static volatile int      s_play_len;     /* number of loaded steps */
static volatile int      s_play_idx;     /* index of the active step */
static volatile bool     s_play_active;  /* playback running */
static volatile bool     s_play_report;  /* playback finished on its own: reporter announces */
static volatile playback_kind_t s_play_kind = PLAY_DUTY;  /* duty vs speed-setpoint steps */
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

/* ISR context: keep it minimal - just wake the owning task for the next tick. */
static bool IRAM_ATTR on_motor_alarm(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx)
{
    BaseType_t high_prio_woken = pdFALSE;
    if (s_motor_task) vTaskNotifyGiveFromISR(s_motor_task, &high_prio_woken);
    return high_prio_woken == pdTRUE;
}

static bool IRAM_ATTR on_imu_alarm(gptimer_handle_t timer,
                                   const gptimer_alarm_event_data_t *edata,
                                   void *user_ctx)
{
    BaseType_t high_prio_woken = pdFALSE;
    if (s_imu_task) vTaskNotifyGiveFromISR(s_imu_task, &high_prio_woken);
    return high_prio_woken == pdTRUE;
}

/* Create + start a periodic GPTimer, stashing the handle so a restart just re-starts it. */
static void start_timer(gptimer_handle_t *out, uint64_t alarm_count,
                        gptimer_alarm_cb_t cb)
{
    gptimer_config_t cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RES_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, out));

    gptimer_event_callbacks_t cbs = { .on_alarm = cb };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(*out, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(*out));

    gptimer_alarm_config_t alarm = {
        .reload_count = 0,
        .alarm_count  = alarm_count,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(*out, &alarm));
    ESP_ERROR_CHECK(gptimer_start(*out));
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

/* Advance the time-based playback script to the step active at now_us. On completion
 * clear s_play_active, flag the reporter and return false with the outputs zeroed. The
 * (vL,vR) values are duty or rad/s per s_play_kind - the caller applies them. */
static bool playback_step(int64_t now_us, float *vL, float *vR)
{
    float elapsed = (float)(now_us - s_play_t0_us) * 1e-6f;
    int i = s_play_idx;
    while (i < s_play_len && elapsed >= s_play[i].t_end) i++;
    s_play_idx = i;
    if (i >= s_play_len) {
        s_play_active = false;
        s_play_report = true;
        *vL = 0.0f; *vR = 0.0f;
        return false;
    }
    *vL = s_play[i].vL;
    *vR = s_play[i].vR;
    return true;
}

static void imu_task(void *arg);   /* spawned by motor_task once peripherals are up */

/* Hard-real-time 500 Hz task (core 1, highest control priority). Owns the encoders,
 * wheel PI and motor output and records telemetry; it never touches I2C, so nothing
 * here can block on the sensor bus. On the first start it brings up the shared
 * peripherals + both GPTimers and spawns imu_task. */
static void motor_task(void *arg)
{
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

    /* Retry the IMU on every (re)start until it succeeds (init is idempotent and also
     * configures the FIFO + GPIO34 INT), so fixing the wiring and restarting brings it
     * up without a reboot. */
    if (!s_imu_present) {
        s_imu_present = mpu6050_init();
        if (!s_imu_present) {
            ESP_LOGW(TAG, "no IMU - balance disabled; motors + encoders still run "
                          "(check wiring at SDA=21 SCL=22, then restart to retry)");
        }
    }

    /* Spawn the IMU task (one priority below this one). Its handle is stored via the
     * out-param before it runs - and before its timer starts below - so on_imu_alarm
     * never notifies a NULL handle. It blocks on its notify until the timer fires. */
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, NULL,
                            configMAX_PRIORITIES - 3, &s_imu_task, 1);

    /* GPTimers pinned to core 1: 250 Hz for the IMU task, 500 Hz here. First start
     * creates them; a restart re-starts the existing timers. */
    if (s_imu_timer == NULL) start_timer(&s_imu_timer, IMU_ALARM_COUNT, on_imu_alarm);
    else                     ESP_ERROR_CHECK(gptimer_start(s_imu_timer));
    if (s_timer == NULL)     start_timer(&s_timer, ALARM_COUNT, on_motor_alarm);
    else                     ESP_ERROR_CHECK(gptimer_start(s_timer));

    /* Ring buffer of the last VEL_WIN ticks' counts + timestamps (windowed speed). */
    int64_t vel_hist_t[VEL_WIN] = { 0 };
    int64_t vel_hist[2][VEL_WIN] = { { 0 } };
    int     vel_head = 0;          /* next slot to write */
    int     vel_n = 0;             /* samples buffered so far (<= VEL_WIN) */
    int64_t last_us = esp_timer_get_time();   /* timestamp of the previous tick */
    bool    primed = false;        /* skip the warning check on the first tick */
    bool    ctrl_was = false;      /* previous tick's controller-enabled state */
    bool    cut_was  = false;      /* previous tick's balance-cut state */
    imu_t   pub_imu = { 0 };       /* last consistent snapshot from imu_task (telemetry) */
    float   pub_roll = NAN, pub_pitch = NAN;
    uint32_t rpt_count = 0;        /* ticks since the last recorded telemetry sample */
    int     active = 0;            /* buffer we are currently writing into */
    int     idx = 0;               /* next slot in the active buffer */

    for (;;) {
        uint32_t nt = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* STOP_CONTROL requested: break at a safe point for cleanup below. */
        if (s_stop_req) break;

        int64_t now_us = esp_timer_get_time();
        int32_t dt_us  = (int32_t)(now_us - last_us);
        last_us = now_us;

        /* nt > 1 means the scheduler didn't run us for a few ticks: run the latest once
         * and count the rest. Period out-of-range only stashes a value for the reporter. */
        if (primed) {
            if (nt > 1) s_motor_missed += nt - 1;
            if (dt_us > DT_MAX_WARN_US || dt_us < DT_MIN_WARN_US) {
                s_warn_dt_us = dt_us;
                s_warn_count++;
            }
        }
        primed = true;

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

        /* Motor commands. Closed loop: the wheel PI, with its setpoint from a speed
         * playback script / balance / the 'speed' command. Open loop: the deadband sweep
         * or a duty playback script take priority, else the effort from the web terminal.
         * ('stop' only zeroes the open-loop command; STOP_CONTROL deletes the task.) */
        float cmdL, cmdR;
        float wsetL = NAN, wsetR = NAN;   /* setpoints for telemetry (NaN when open loop) */
        float rawL = NAN, rawR = NAN;         /* raw PI output pre-deadband/sat (NaN when open loop) */
        if (s_ctrl_enabled) {
            /* Rising edge into closed loop: clear integrators/filters for a clean start. */
            if (!ctrl_was) wheel_pi_reset();
            float dt = (dt_us > 0) ? (float)dt_us * 1e-6f : (1.0f / (float)CONTROL_HZ);

            /* Setpoint source, in priority order:
             *  1. a running speed playback script (reproducible step input for the PI),
             *  2. the balance loop's published common setpoint (ZOH from imu_task),
             *  3. otherwise the last 'speed' command persists.
             * On a balance cut, zero the wheels and reset the PI once so it doesn't wind
             * up while parked. */
            bool balance_cut = false;
            if (s_play_active && s_play_kind == PLAY_SPEED && !s_bal_enabled) {
                float vL, vR;
                playback_step(now_us, &vL, &vR);   /* sets 0 on completion */
                wheel_pi_set_setpoint(0, vL);
                wheel_pi_set_setpoint(1, vR);
            } else if (s_bal_enabled) {
                if (s_bal_cut) {
                    balance_cut = true;
                } else {
                    float w = s_bal_w_common;
                    wheel_pi_set_setpoint(0, w);
                    wheel_pi_set_setpoint(1, w);
                }
            }

            if (balance_cut) {
                if (!cut_was) wheel_pi_reset();
                cmdL = 0.0f;
                cmdR = 0.0f;
            } else {
                /* Per-wheel PI + deadband + anti-windup -> duty; dt is the measured
                 * period so gains stay correct under jitter. Setpoints come from the
                 * balance loop (above) or the 'speed' command. */
                wsetL = wheel_pi_setpoint(0);
                wsetR = wheel_pi_setpoint(1);
                cmdL = wheel_pi_step(0, velL, dt);
                cmdR = wheel_pi_step(1, velR, dt);
                rawL = wheel_pi_raw(0);
                rawR = wheel_pi_raw(1);
            }
            cut_was = balance_cut;
        } else if (s_db_active) {
            /* Deadband sweep takes priority over playback/manual while running. */
            deadband_step(now_us, &cmdL, &cmdR);
        } else if (s_play_active && s_play_kind == PLAY_DUTY) {
            /* Open-loop duty playback: apply the current step; on completion also park
             * the manual command so the wheels don't jump to a stale 'motor' value. */
            float vL, vR;
            if (!playback_step(now_us, &vL, &vR)) {
                motor_cmd_set(0, 0.0f);
                motor_cmd_set(1, 0.0f);
            }
            cmdL = vL;
            cmdR = vR;
        } else {
            cmdL = motor_cmd_get(0);
            cmdR = motor_cmd_get(1);
        }
        motor_set(0, cmdL);
        motor_set(1, cmdR);
        ctrl_was = s_ctrl_enabled;

        /* Record one telemetry sample every REPORT_DIV-th tick. REPORT_DIV = 1 records at
         * the full loop rate so the motors topic can stream at CONTROL_HZ; the reporter
         * decimates the other topics per mode. The IMU sample + estimate come from
         * imu_task via the seqlock; a torn read keeps the previous snapshot. */
        if (++rpt_count >= REPORT_DIV) {
            rpt_count = 0;
            imu_pub_try_read(&pub_imu, &pub_roll, &pub_pitch);
            sample_t *smp = &s_buf[active][idx];
            smp->t_us = now_us;
            smp->posL = (float)encoder_angle_rad(0);
            smp->posR = (float)encoder_angle_rad(1);
            smp->velL = velL;
            smp->velR = velR;
            smp->roll  = pub_roll;
            smp->pitch = pub_pitch;
            smp->wsetL = wsetL;
            smp->wsetR = wsetR;
            smp->rawL   = rawL;
            smp->rawR   = rawR;
            smp->mL   = cmdL;
            smp->mR   = cmdR;
            smp->imu  = pub_imu;

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

        /* Compute-budget watchdog: time this tick's work; sum it for the average and
         * keep the worst for the reporter. Over RUN_WARN_US (an absolute ceiling well
         * above the lean motor loop's norm) flags a real compute regression. */
        int32_t run_us = (int32_t)(esp_timer_get_time() - now_us);
        s_motor_run_sum += (uint32_t)run_us;
        s_motor_run_cnt++;
        if (run_us > RUN_WARN_US) {
            if (run_us > s_warn_run_us) s_warn_run_us = run_us;
            s_warn_run_count++;
        }
    }

    /* STOP: stop the 500 Hz timer (so its ISR can't notify a deleted task), park the
     * motors, then self-delete. imu_task stops its own timer and exits on s_stop_req. */
    ESP_ERROR_CHECK(gptimer_stop(s_timer));
    motor_set(0, 0.0f);
    motor_set(1, 0.0f);
    s_motor_task = NULL;
    vTaskDelete(NULL);
}

/* 250 Hz task (core 1, one priority below motor_task). Owns the blocking MPU6050 read,
 * the complementary filter and the balance PID; it publishes the common wheel-speed
 * setpoint + fall-cut for the motor task and a telemetry snapshot. Running below
 * motor_task means its I2C wait/compute is always preempted and can never delay a motor
 * tick - the whole point of the split. */
static void imu_task(void *arg)
{
    imu_t    imu = { 0 };                      /* last good sample (held across stale ticks) */
    int64_t  last_us = esp_timer_get_time();
    uint32_t last_dr = mpu6050_dr_count();     /* data-ready edges seen so far */
    uint32_t stale_streak = 0;                 /* consecutive ticks with no fresh sample */
    bool     primed = false;
    bool     bal_was = false;
    uint32_t bal_div = 0;                       /* imu ticks since the last balance step */
    float    bal_dt  = 0.0f;                    /* elapsed time accumulated for that step, s */

    for (;;) {
        uint32_t nt = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_stop_req) break;

        int64_t now_us = esp_timer_get_time();
        int32_t dt_us  = (int32_t)(now_us - last_us);
        last_us = now_us;
        if (primed && nt > 1) s_imu_missed += nt - 1;   /* ran late: count skipped ticks */
        primed = true;
        float dt = (dt_us > 0) ? (float)dt_us * 1e-6f : (1.0f / (float)IMU_HZ);

        /* New samples produced since last tick, straight from the GPIO34 INT counter -
         * no I2C. Zero => no fresh data this tick (hold last, flag stale). */
        uint32_t dr = mpu6050_dr_count();
        uint32_t newn = dr - last_dr;
        last_dr = dr;

        if (!s_imu_present) {
            imu.ok = false;
        } else if (newn == 0) {
            imu.ok = false;
            s_imu_stale_count++;
            /* A long run with no data-ready edge means the IMU is gone (unplugged/dead):
             * drop it (balance disables); the reporter surfaces it and a restart re-inits. */
            if (++stale_streak > IMU_FAIL_LIMIT) {
                s_imu_present = false; s_imu_lost_evt = true; stale_streak = 0;
            }
        } else {
            if (newn > 1) s_imu_skipped += newn - 1;   /* older frames dropped (take-newest) */
            int64_t t0 = esp_timer_get_time();
            imu_t fresh;
            bool  ok = mpu6050_read_newest(&fresh);
            int32_t rus = (int32_t)(esp_timer_get_time() - t0);
            if (rus > s_warn_imu_us) s_warn_imu_us = rus;
            if (ok && fresh.ok) {
                imu = fresh;
                stale_streak = 0;
            } else {
                imu.ok = false;
                s_imu_stale_count++;
                if (++stale_streak > IMU_FAIL_LIMIT) {
                    s_imu_present = false; s_imu_lost_evt = true; stale_streak = 0;
                }
            }
        }

        // Gyro-bias calibration when armed; before the estimator so it re-seeds same tick.
        if (s_gc_active) gyrocal_step(imu);

        /* Complementary-filter tilt estimate. NaN when estimation is off, no IMU, or not
         * yet seeded. dt is the measured period (rate-independent alpha handles it). */
        float roll = NAN, pitch = NAN;
        if (!s_est_enabled || !s_imu_present) {
            estimator_reset();
        } else if (!estimator_update(imu, dt, &roll, &pitch)) {
            roll = NAN; pitch = NAN;   /* not seeded yet */
        }

        /* Publish the estimate + IMU sample for the motor task's telemetry record. */
        imu_pub_write(&imu, roll, pitch);

        /* Balance (outer) loop -> common wheel-speed setpoint + fall-cut, consumed by the
         * motor task. It runs at BALANCE_HZ (every BALANCE_DIV-th tick): accumulate the
         * elapsed time and step the PID with that true dt so the integral gain stays
         * correct; between steps the last w_common/cut hold (the motor task ZOH-consumes
         * them anyway). Above BALANCE_MAX_TILT the catch can't recover, so cut. Forcing the
         * cut when disabled keeps a stale setpoint from leaking on the next enable. */
        if (s_bal_enabled) {
            if (!bal_was) {                      /* rising edge: clear integrator, resync sub-rate */
                balance_pid_reset();
                bal_div = 0;
                bal_dt  = 0.0f;
            }
            bal_dt += dt;
            if (++bal_div >= BALANCE_DIV) {      /* BALANCE_HZ step */
                bool have_theta = s_imu_present && s_est_enabled && isfinite(pitch);
                if (have_theta && fabsf(pitch) < BALANCE_MAX_TILT) {
                    float theta_dot = imu.gy * DEG2RAD;   /* tilt rate about Y, rad/s */
                    s_bal_w_common = balance_pid_step(pitch, theta_dot, bal_dt);
                    s_bal_cut = false;
                } else {
                    balance_pid_reset();
                    s_bal_w_common = 0.0f;
                    s_bal_cut = true;
                }
                bal_div = 0;
                bal_dt  = 0.0f;
            }
        } else {
            s_bal_w_common = 0.0f;
            s_bal_cut = true;
            bal_div = 0;                          /* resync so the first enabled step waits a full period */
            bal_dt  = 0.0f;
        }
        bal_was = s_bal_enabled;

        /* Sum this tick's run time (includes the blocking I2C read) + new-sample count
         * for the reporter's rolling averages. */
        s_imu_run_sum  += (uint32_t)(esp_timer_get_time() - now_us);
        s_imu_run_cnt++;
        s_imu_samp_sum += newn;
    }

    /* STOP: stop the 250 Hz timer, then self-delete. */
    ESP_ERROR_CHECK(gptimer_stop(s_imu_timer));
    s_imu_task = NULL;
    vTaskDelete(NULL);
}

/* Announce any timing violations since the last batch to the console and web
 * terminal. Coalesces a burst into one message with a count. */
static void emit_pending_warning(void)
{
    char msg[192];
    char json[288];

    /* Motor loop: period out of range (tick scheduled early/late) and/or missed ticks
     * (ran late, coalesced). Both live on the 500 Hz motor task. */
    uint32_t n     = s_warn_count;
    uint32_t mmiss = s_motor_missed;
    if (n != 0 || mmiss != 0) {
        int32_t dt = s_warn_dt_us;
        s_warn_count   = 0;      /* clear before reporting; races are harmless */
        s_motor_missed = 0;
        snprintf(msg, sizeof(msg),
                 "motor loop timing: %" PRIu32 " out-of-range, %" PRIu32 " missed tick(s) "
                 "in the last %d ms, latest dt=%ld us (want ~%d, limits %d..%d)",
                 n, mmiss, BATCH_MS, (long)dt,
                 CONTROL_PERIOD_US, DT_MIN_WARN_US, DT_MAX_WARN_US);
        ESP_LOGW(TAG, "%s", msg);
        snprintf(json, sizeof(json), "{\"type\":\"warn\",\"text\":\"%s\"}", msg);
        ws_broadcast(json);
    }

    /* Motor compute budget exceeded. The blocking IMU read now lives in imu_task, so
     * this is pure motor-loop compute (encoders + wheel PI + telemetry). */
    uint32_t rn = s_warn_run_count;
    if (rn != 0) {
        int32_t run = s_warn_run_us;
        s_warn_run_count = 0;
        s_warn_run_us    = 0;    /* reset the worst-case tracker for next window */
        snprintf(msg, sizeof(msg),
                 "motor task over budget: %" PRIu32 " tick(s) in the last %d ms, "
                 "worst run=%ld us (limit %d us, period %d us)",
                 rn, BATCH_MS, (long)run, RUN_WARN_US, CONTROL_PERIOD_US);
        ESP_LOGW(TAG, "%s", msg);
        snprintf(json, sizeof(json), "{\"type\":\"warn\",\"text\":\"%s\"}", msg);
        ws_broadcast(json);
    }

    /* IMU task health: missed 250 Hz ticks, stale ticks (no fresh sample; held last)
     * and older FIFO frames skipped (take-newest). Emitted only on an anomaly; the
     * skip/worst-read counters are always reset so they never accumulate. */
    uint32_t imiss     = s_imu_missed;
    uint32_t stale     = s_imu_stale_count;
    uint32_t skipped   = s_imu_skipped;
    int32_t  imu_worst = s_warn_imu_us;
    s_imu_missed      = 0;
    s_imu_stale_count = 0;
    s_imu_skipped     = 0;
    s_warn_imu_us     = 0;
    if (imiss != 0 || stale != 0) {
        snprintf(msg, sizeof(msg),
                 "IMU task: %" PRIu32 " missed, %" PRIu32 " stale (held last), "
                 "%" PRIu32 " sample(s) skipped in the last %d ms; worst read=%ld us",
                 imiss, stale, skipped, BATCH_MS, (long)imu_worst);
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

    /* IMU dropped after a long run with no fresh samples: balance is now disabled until
     * a restart. Most often the data-ready INT wire (GPIO34) is missing, since that is
     * the liveness signal - so point at it as well as a dead/unplugged sensor. */
    if (s_imu_lost_evt) {
        s_imu_lost_evt = false;
        snprintf(msg, sizeof(msg),
                 "IMU lost: no data-ready edges for %d ticks - balance disabled; check the "
                 "MPU INT wire to GPIO34 (and SDA=21 SCL=22), then restart to retry",
                 IMU_FAIL_LIMIT);
        ESP_LOGW(TAG, "%s", msg);
        snprintf(json, sizeof(json), "{\"type\":\"error\",\"text\":\"%s\"}", msg);
        ws_broadcast(json);
    }
}

/* Rolling loop-timing averages, ~1 Hz. Averages + resets the per-tick accumulators the
 * two tasks fill, publishes them for the stats command, and prints one info line so an
 * occasional out-of-range period can be judged against the norm - the average run time
 * normally sits well under budget even when a single tick is scheduled early/late. */
static void emit_loop_stats(void)
{
    uint32_t mcnt = s_motor_run_cnt, msum = s_motor_run_sum;
    uint32_t icnt = s_imu_run_cnt,   isum = s_imu_run_sum, isamp = s_imu_samp_sum;
    s_motor_run_cnt = 0; s_motor_run_sum = 0;
    s_imu_run_cnt   = 0; s_imu_run_sum   = 0; s_imu_samp_sum = 0;

    float motor_us = mcnt ? (float)msum / (float)mcnt : 0.0f;
    float imu_us   = icnt ? (float)isum / (float)icnt : 0.0f;
    float samples  = icnt ? (float)isamp / (float)icnt : 0.0f;
    s_stat_motor_run_us = motor_us;
    s_stat_imu_run_us   = imu_us;
    s_stat_imu_samples  = samples;

    char msg[192];
    snprintf(msg, sizeof(msg),
             "loop avg over %d ms: motor run=%.0f us, imu run=%.0f us, "
             "imu samples/tick=%.2f",
             STATS_MS, (double)motor_us, (double)imu_us, (double)samples);
    ESP_LOGI(TAG, "%s", msg);
    char json[256];
    snprintf(json, sizeof(json), "{\"type\":\"resp\",\"text\":\"%s\"}", msg);
    ws_broadcast(json);
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
    snprintf(msg, sizeof(msg), "playback finished (%d steps, %.2f s), %s",
             s_play_len, (double)total,
             s_play_kind == PLAY_SPEED ? "setpoints zeroed" : "motors parked");
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

        /* Publish the rolling loop-timing averages (~1 Hz), for context on the above. */
        static uint32_t stats_batches = 0;
        if (++stats_batches >= STATS_DIV) {
            stats_batches = 0;
            emit_loop_stats();
        }

        /* Announce the deadband sweep result when the loop has finished one. */
        emit_deadband_result();

        /* Announce when a playback script has run to completion. */
        emit_playback_result();

        /* Announce the gyro-bias calibration result when the loop finishes one. */
        emit_gyrocal_result();

        /* Pack one binary frame per topic and stream it, decimating per topic (recording
         * is full-rate). motors streams tick-by-tick (CONTROL_HZ) in TEST_MOTOR_CONTROLLERS
         * so a wheel step is captured exactly, else at STREAM_HZ; angles is dropped in that
         * mode (not needed while tuning wheels); imu is one newest sample per second. The
         * scratch buffer is static (too big for the task stack, and avoids the heap
         * fragmentation a per-batch malloc caused). Only reporter_task touches it. */
        if (web_server_streaming()) {
            static uint8_t frame[REPORT_FRAME_CAP];
            static uint32_t batch_no = 0;
            bool motor_ctrl = s_ctrl_enabled && !s_est_enabled && !s_bal_enabled;
            for (int t = 0; t < telemetry_topic_count(); t++) {
                const char     *name = telemetry_topic_name(t);
                const sample_t *src  = batch;
                int count, stride;
                if (strcmp(name, "motors") == 0) {
                    stride = motor_ctrl ? 1 : STREAM_DECIM;   /* CONTROL_HZ vs STREAM_HZ */
                    count  = SAMPLES_PER_BATCH / stride;
                } else if (strcmp(name, "angles") == 0) {
                    if (motor_ctrl) continue;                 /* not needed while tuning wheels */
                    stride = STREAM_DECIM;
                    count  = SAMPLES_PER_BATCH / stride;
                } else {   /* imu: one newest sample per second */
                    if ((batch_no % IMU_STREAM_DIV) != 0) continue;
                    src    = &batch[SAMPLES_PER_BATCH - 1];
                    stride = 1;
                    count  = 1;
                }
                int n = telemetry_topic_pack(frame, sizeof(frame), t, src, count, stride);
                if (n > 0) {
                    ws_broadcast_bin(frame, (size_t)n);
                } else {
                    ESP_LOGW(TAG, "topic '%s' frame did not fit in %u bytes",
                             name, (unsigned)sizeof(frame));
                }
            }
            batch_no++;
        }
    }
}

void control_start(void)
{
    /* Ready-buffer index queue. Length 2 so the motor task can hand over even
     * while the reporter is mid-cycle on the previous buffer. */
    s_ready_q = xQueueCreate(2, sizeof(int));
    configASSERT(s_ready_q);

    /* Reporter on core 0 (shares with Wi-Fi). If Wi-Fi/WebSocket stalls, only this
     * task waits; the control loops are unaffected. */
    xTaskCreatePinnedToCore(reporter_task, "reporter", 4096, NULL, 4, NULL, 0);

    /* Start the motor task (created on core 1; it brings up peripherals + both GPTimers
     * there and spawns the IMU task). */
    control_set_mode(START_CONTROL);
}

control_mode_t control_mode(void) { return s_mode; }

void control_loop_stats(float *motor_run_us, float *imu_run_us, float *imu_samples)
{
    if (motor_run_us) *motor_run_us = s_stat_motor_run_us;
    if (imu_run_us)   *imu_run_us   = s_stat_imu_run_us;
    if (imu_samples)  *imu_samples  = s_stat_imu_samples;
}

void control_set_mode(control_mode_t mode)
{
    if (mode == START_CONTROL) {
        if (s_mode == START_CONTROL) return;          /* already running */
        s_stop_req = false;
        s_mode     = START_CONTROL;
        /* Motor loop on core 1, high priority, isolated from core 0's Wi-Fi. It brings up
         * peripherals + both GPTimers and spawns imu_task. Its handle is set via the
         * out-param before it runs, so on_motor_alarm never notifies a NULL handle. */
        xTaskCreatePinnedToCore(motor_task, "motor", 4096, NULL,
                                configMAX_PRIORITIES - 2, &s_motor_task, 1);
    } else {  /* STOP_CONTROL */
        if (s_mode == STOP_CONTROL) return;           /* already stopped */
        /* Ask both loops to exit at a safe point (each stops its own timer; motor parks
         * the motors, self-delete), then wait (bounded) until both are gone so callers
         * know control stopped on return. */
        s_stop_req = true;
        for (int i = 0; i < 200 && (s_motor_task != NULL || s_imu_task != NULL); i++) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        s_stop_req = false;
        s_mode     = STOP_CONTROL;
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

void control_playback_begin(playback_kind_t kind)
{
    s_play_active = false;   /* stop the loop touching it before we refill */
    s_play_idx = 0;
    s_play_len = 0;
    s_play_kind = kind;
}

int control_playback_append(float dur, float vL, float vR)
{
    if (s_play_len >= PLAYBACK_MAX) return -1;
    if (dur <= 0.0f) return -2;             /* a step must last a positive time */
    int i = s_play_len;
    float prev_end = (i > 0) ? s_play[i - 1].t_end : 0.0f;
    if (prev_end + dur > PLAYBACK_MAX_S) return -3;   /* would exceed the total-time cap */
    /* Clamp to the kind's range: duty to the drive limit, speed to the setpoint cap. */
    float lim = (s_play_kind == PLAY_SPEED) ? WHEEL_PI_WSET_MAX : 1.0f;
    if (vL >  lim) vL =  lim; else if (vL < -lim) vL = -lim;
    if (vR >  lim) vR =  lim; else if (vR < -lim) vR = -lim;
    s_play[i].t_end = prev_end + dur;       /* durations -> cumulative end time */
    s_play[i].vL    = vL;
    s_play[i].vR    = vR;
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
    wheel_pi_set_setpoint(0, 0.0f);   /* also drop a speed-script setpoint */
    wheel_pi_set_setpoint(1, 0.0f);
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
