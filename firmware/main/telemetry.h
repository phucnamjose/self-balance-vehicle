/**
 * @file telemetry.h
 * @brief Shared loop-state types and the cached snapshot used by the web server.
 *
 * The control loop writes one sample_t every tick into a double buffer and
 * checks each tick's measured period against the limits below, flagging a
 * warning if it runs long or short. After SAMPLES_PER_BATCH samples (0.5 s) it
 * swaps buffers and hands the full one to the reporter task, which emits any
 * pending timing warning, caches the latest sample for HTTP/WebSocket readers,
 * and streams the whole batch to the browser to be saved as a file.
 */
#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* Control loop rate (Hz). Shared because telemetry tags each frame with it. */
#define CONTROL_HZ     200

/* Nominal loop period and how many samples make up one reported batch. At
 * 200 Hz a half-second batch is 100 samples. */
#define CONTROL_PERIOD_US   (1000000 / CONTROL_HZ)
#define SAMPLES_PER_BATCH   (CONTROL_HZ / 2)

/* Timing limits for the warning job (tunable). The control loop warns when a
 * tick's measured period runs longer than DT_MAX_WARN_US or shorter than
 * DT_MIN_WARN_US. */
#define DT_MAX_WARN_US (CONTROL_PERIOD_US * 7 / 5)                   /* +40% */
#define DT_MIN_WARN_US (CONTROL_PERIOD_US * 3 / 5)                   /* -40% */

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
    float   mL, mR;        /* commanded motor effort, -1..+1 */
    imu_t   imu;           /* latest IMU sample */
} sample_t;

/* Create the mutex guarding the cached latest sample. Call once at boot. */
void telemetry_init(void);

/* Replace the cached latest sample (called by the reporter task). */
void telemetry_set_latest(const sample_t *s);

/* Serialize the cached latest sample to JSON. @p type tags the message so the
 * client can tell telemetry from command replies. Returns the string length. */
int telemetry_latest_json(char *buf, size_t len, const char *type);

/* Telemetry is split into named topics (imu / angles / motors), each a
 * projection of sample_t with its own column set. The reporter streams one
 * frame per topic so the client can record each to its own file. */
int         telemetry_topic_count(void);
const char *telemetry_topic_name(int topic);

/* Pack @p n samples of topic @p topic into @p buf as a compact binary frame
 * (sent as a WebSocket binary message):
 *
 *   [u8 topic_id][u8 field_count][u16 sample_count]   (little-endian)
 *   then, per sample: uint32 t_ms, followed by (field_count-1) float32 values
 *
 * Column order/units are documented next to each topic's packer in telemetry.c
 * and mirrored on the client. Not-yet-available fields are packed as NaN.
 * Returns the byte length, or a negative value if it would not fit. */
int telemetry_topic_pack(uint8_t *buf, size_t len, int topic,
                         const sample_t *samples, int n);
