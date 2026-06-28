#include "telemetry.h"

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Latest snapshot cached for the HTTP server, guarded by a mutex because the
 * web server task and reporter_task touch it from different contexts. */
static telemetry_t      s_latest;
static SemaphoreHandle_t s_latest_lock;

void telemetry_init(void)
{
    s_latest_lock = xSemaphoreCreateMutex();
    configASSERT(s_latest_lock);
}

void telemetry_set(const telemetry_t *snap)
{
    xSemaphoreTake(s_latest_lock, portMAX_DELAY);
    s_latest = *snap;
    xSemaphoreGive(s_latest_lock);
}

int telemetry_to_json(char *buf, size_t len, const char *type)
{
    telemetry_t snap;
    xSemaphoreTake(s_latest_lock, portMAX_DELAY);
    snap = s_latest;
    xSemaphoreGive(s_latest_lock);

    return snprintf(buf, len,
        "{\"type\":\"%s\",\"rate\":%d,\"ticks\":%" PRIu32 ",\"dt_avg\":%lld,"
        "\"dt_min\":%lld,\"dt_max\":%lld,\"mL\":%.2f,\"mR\":%.2f,"
        "\"posL\":%lld,\"posR\":%lld,\"velL\":%ld,\"velR\":%ld,"
        "\"imu_ok\":%d,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"temp\":%.1f}",
        type, CONTROL_HZ, snap.ticks, (long long)snap.dt_avg_us,
        (long long)snap.dt_min_us, (long long)snap.dt_max_us,
        snap.mL, snap.mR,
        (long long)snap.posL, (long long)snap.posR,
        (long)snap.velL, (long)snap.velR,
        snap.imu.ok ? 1 : 0,
        snap.imu.ax, snap.imu.ay, snap.imu.az,
        snap.imu.gx, snap.imu.gy, snap.imu.gz, snap.imu.temp_c);
}
