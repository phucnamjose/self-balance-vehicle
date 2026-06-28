/**
 * @file telemetry.h
 * @brief Shared loop-state types and the cached snapshot used by the web server.
 *
 * The control loop produces a telemetry_t each second; the reporter caches the
 * latest one here (guarded by a mutex) so HTTP/WebSocket handlers on the other
 * core can read it and serialize it to JSON.
 */
#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* Control loop rate (Hz). Shared because telemetry tags each frame with it. */
#define CONTROL_HZ     200

/* One IMU sample in physical units. */
typedef struct {
    bool  ok;              /* false if the last read failed */
    float ax, ay, az;      /* acceleration in g */
    float gx, gy, gz;      /* angular rate in deg/s */
    float temp_c;          /* die temperature in degC */
} imu_t;

/* Snapshot of loop state, produced by control_task and consumed by reporter_task. */
typedef struct {
    uint32_t ticks;        /* total control ticks so far */
    int64_t  dt_avg_us;    /* average loop period over the last window */
    int64_t  dt_min_us;    /* min/max loop period (jitter) over the window */
    int64_t  dt_max_us;
    float    mL, mR;       /* commanded motor effort, -1..+1 */
    int64_t  posL, posR;   /* total encoder counts */
    int32_t  velL, velR;   /* encoder counts/sec over the last window */
    imu_t    imu;          /* latest IMU sample */
} telemetry_t;

/* Create the mutex guarding the cached snapshot. Call once at boot. */
void telemetry_init(void);

/* Replace the cached snapshot (called by the reporter task). */
void telemetry_set(const telemetry_t *snap);

/* Serialize the cached snapshot to JSON. @p type tags the message so the client
 * can tell telemetry from command replies. Returns the string length. */
int telemetry_to_json(char *buf, size_t len, const char *type);
