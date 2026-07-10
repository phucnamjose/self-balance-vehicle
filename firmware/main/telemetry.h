/**
 * @file telemetry.h
 * @brief Shared loop-state types and the cached snapshot used by the web server.
 *
 * The control loop writes one sample_t per tick into a double buffer; after
 * SAMPLES_PER_BATCH samples (0.2 s) it hands the full buffer to the reporter,
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

/* Loop period and batch size, derived from the rate so a batch is always 200 ms. */
#define CONTROL_PERIOD_US   (1000000 / CONTROL_HZ)
#define SAMPLES_PER_BATCH   (CONTROL_HZ / 5)

/* Period warning band, an ABSOLUTE margin (not a % of the period): jitter sources
 * are absolute in time (scheduler wake, a ~1 ms I2C stall) and don't scale with
 * the rate. DT_JITTER_MARGIN_US covers one catch-up after a slow tick; at 500 Hz
 * this gives a 1.0..3.0 ms window. */
#define DT_JITTER_MARGIN_US 1000
#define DT_MAX_WARN_US (CONTROL_PERIOD_US + DT_JITTER_MARGIN_US)
#define DT_MIN_WARN_US (CONTROL_PERIOD_US - DT_JITTER_MARGIN_US)

/* Compute-budget limit: how long the tick's WORK takes (vs the period above). Warn
 * once it exceeds 90% of the period (1.8 ms at 500 Hz), or the next tick starts late. */
#define RUN_WARN_US    (CONTROL_PERIOD_US * 9 / 10)                  /* 90% */

/* One IMU sample in physical units. */
typedef struct {
    bool  ok;              /* false if the last read failed */
    float ax, ay, az;      /* acceleration in g */
    float gx, gy, gz;      /* angular rate in deg/s */
    float temp_c;          /* die temperature in degC */
} imu_t;

/* One per-tick sample, produced by control_task at CONTROL_HZ. */
typedef struct {
    int64_t t_us;          /* esp_timer_get_time() at this tick (real time) */
    float   posL, posR;    /* wheel angle, radians */
    float   velL, velR;    /* wheel angular speed, rad/s */
    float   roll, pitch;   /* orientation estimate, rad (NaN when estimation off) */
    float   wsetL, wsetR;  /* per-wheel speed setpoint, rad/s (NaN when open loop) */
    float   uL, uR;        /* raw PI output pre-deadband/sat (NaN when open loop) */
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

/* Pack @p n samples of @p topic into @p buf as a little-endian binary frame:
 *   [u8 topic_id][u8 field_count][u16 sample_count] then per-sample u64 t_us +
 *   (field_count-1) float32. Column order matches telemetry.c and the client.
 * Returns the byte length, or negative if it would not fit. */
int telemetry_topic_pack(uint8_t *buf, size_t len, int topic,
                         const sample_t *samples, int n);
