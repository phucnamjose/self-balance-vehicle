/**
 * @file telemetry.h
 * @brief Shared loop-state types and the cached snapshot used by the web server.
 *
 * The motor loop writes one sample_t every REPORT_DIV ticks into a double buffer;
 * after SAMPLES_PER_BATCH samples (0.2 s) it hands the full buffer to the reporter,
 * which caches the latest sample and streams the batch to the browser.
 */
#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* Control loop rate (Hz), shared so telemetry can tag each frame. 500 Hz keeps
 * sampling/ZOH dead time low for the balance limiter; the IMU DLPF and the
 * read->compute->PWM path now dominate latency. Rate-dependent constants
 * (COMP_ALPHA_DEFAULT, VEL_WIN) are preserved in time when this changes. */
#define CONTROL_HZ     500

/* Telemetry is recorded every REPORT_DIV-th tick. REPORT_DIV = 1 records at the full
 * loop rate so the motors topic can stream tick-by-tick (CONTROL_HZ); the reporter then
 * decimates each topic at stream time (STREAM_DECIM / IMU_STREAM_DIV) to trim bandwidth. */
#define REPORT_DIV     1
#define REPORT_HZ      (CONTROL_HZ / REPORT_DIV)

/* Default stream rate for the decimated topics (motors outside TEST_MOTOR_CONTROLLERS,
 * and angles): keep every STREAM_DECIM-th recorded sample. Must divide SAMPLES_PER_BATCH. */
#define STREAM_HZ      100
#define STREAM_DECIM   (CONTROL_HZ / STREAM_HZ)

/* Loop period and batch size. A batch holds SAMPLES_PER_BATCH telemetry samples
 * (0.2 s at REPORT_HZ); BATCH_MS is that span, used for the warning windows. */
#define CONTROL_PERIOD_US   (1000000 / CONTROL_HZ)
#define SAMPLES_PER_BATCH   (REPORT_HZ / 5)
#define BATCH_MS            (1000 * SAMPLES_PER_BATCH / REPORT_HZ)

/* Period warning band, an ABSOLUTE margin (not a % of the period): jitter sources
 * are absolute in time (scheduler wake, a ~1 ms I2C stall) and don't scale with
 * the rate. DT_JITTER_MARGIN_US covers one catch-up after a slow tick; at 500 Hz
 * this gives a 1.0..3.0 ms window. */
#define DT_JITTER_MARGIN_US 1000
#define DT_MAX_WARN_US (CONTROL_PERIOD_US + DT_JITTER_MARGIN_US)
#define DT_MIN_WARN_US (CONTROL_PERIOD_US - DT_JITTER_MARGIN_US)

/* Motor-task compute-budget limit [us]: warn if a 500 Hz tick's WORK exceeds this. The
 * motor loop no longer does the blocking IMU read (that moved to imu_task), so its work
 * is only tens of us - an absolute ceiling well above the norm flags a real compute
 * regression, separate from the scheduling jitter the DT_* period band already covers. */
#define RUN_WARN_US    500

/* One IMU sample in physical units. */
typedef struct {
    bool  ok;              /* false if the last read failed */
    float ax, ay, az;      /* acceleration in g */
    float gx, gy, gz;      /* angular rate in deg/s */
    float temp_c;          /* die temperature in degC */
} imu_t;

/* One recorded sample, produced by motor_task at REPORT_HZ. */
typedef struct {
    int64_t t_us;          /* esp_timer_get_time() at this tick (real time) */
    float   posL, posR;    /* wheel angle, radians */
    float   velL, velR;    /* wheel angular speed, rad/s */
    float   roll, pitch;   /* orientation estimate, rad (NaN when estimation off) */
    float   wsetL, wsetR;  /* per-wheel speed setpoint, rad/s (NaN when open loop) */
    float   rawL, rawR;        /* raw PI output pre-deadband/sat (NaN when open loop) */
    float   mL, mR;        /* commanded motor effort, -1..+1 */
    imu_t   imu;           /* latest IMU sample */
} sample_t;

/* Create the mutex guarding the cached latest sample. Call once at boot. */
void telemetry_init(void);

/* Replace the cached latest sample (called by the reporter task). */
void telemetry_set_latest(const sample_t *s);

/* Serialize the cached latest sample to JSON. @p type tags the message (telemetry
 * vs command reply). Returns the string length. */
int telemetry_latest_json(char *buf, size_t len, const char *type);

/* Telemetry is split into named topics (imu / angles / motors), each a projection
 * of sample_t with its own column set, streamed as one frame per topic. */
int         telemetry_topic_count(void);
const char *telemetry_topic_name(int topic);

/* Pack @p count samples of @p topic into @p buf as a little-endian binary frame:
 *   [u8 topic_id][u8 field_count][u16 sample_count] then per-sample u64 t_us +
 *   (field_count-1) float32. Samples are taken with @p stride (samples[i*stride],
 *   i in [0,count)) so a topic can be decimated. Column order matches telemetry.c and
 *   the client. Returns the byte length, or negative if it would not fit. */
int telemetry_topic_pack(uint8_t *buf, size_t len, int topic,
                         const sample_t *samples, int count, int stride);
