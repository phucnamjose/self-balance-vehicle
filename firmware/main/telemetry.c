#include "telemetry.h"

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Latest sample cached for the HTTP server, guarded by a mutex because the web
 * server task and reporter_task touch it from different contexts. */
static sample_t          s_latest;
static SemaphoreHandle_t s_latest_lock;

void telemetry_init(void)
{
    s_latest_lock = xSemaphoreCreateMutex();
    configASSERT(s_latest_lock);
}

void telemetry_set_latest(const sample_t *s)
{
    xSemaphoreTake(s_latest_lock, portMAX_DELAY);
    s_latest = *s;
    xSemaphoreGive(s_latest_lock);
}

int telemetry_latest_json(char *buf, size_t len, const char *type)
{
    sample_t s;
    xSemaphoreTake(s_latest_lock, portMAX_DELAY);
    s = s_latest;
    xSemaphoreGive(s_latest_lock);

    return snprintf(buf, len,
        "{\"type\":\"%s\",\"rate\":%d,\"t\":%.3f,"
        "\"mL\":%.2f,\"mR\":%.2f,"
        "\"posL\":%.4f,\"posR\":%.4f,\"velL\":%.3f,\"velR\":%.3f,"
        "\"imu_ok\":%d,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"temp\":%.1f}",
        type, CONTROL_HZ, (double)s.t_us / 1e6,
        s.mL, s.mR, s.posL, s.posR, s.velL, s.velR,
        s.imu.ok ? 1 : 0,
        s.imu.ax, s.imu.ay, s.imu.az,
        s.imu.gx, s.imu.gy, s.imu.gz, s.imu.temp_c);
}

/* ============================== topics ==================================
 * Each topic is a projection of sample_t: a name plus a row encoder that prints
 * one "[..]" array. Rows omit keys to stay small, so the column order documented
 * above each encoder is the contract the client decoder must mirror. Fields that
 * are not produced yet (orientation estimate, control setpoints) are emitted as
 * the JSON literal null and become empty cells in the CSV. */
typedef int (*row_fn)(char *buf, size_t len, const sample_t *s);

/* imu: [ t, ax, ay, az, gx, gy, gz, temp, ok ]
 *   t            seconds since boot
 *   ax,ay,az     acceleration, g
 *   gx,gy,gz     angular rate, deg/s
 *   temp         IMU die temperature, degC
 *   ok           1 if the IMU read succeeded, else 0 */
static int row_imu(char *b, size_t n, const sample_t *s)
{
    return snprintf(b, n, "[%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%d]",
        (double)s->t_us / 1e6,
        s->imu.ax, s->imu.ay, s->imu.az,
        s->imu.gx, s->imu.gy, s->imu.gz, s->imu.temp_c,
        s->imu.ok ? 1 : 0);
}

/* angles: [ t, roll, pitch, roll_sp, pitch_sp ]  (radians)
 *   roll,pitch        measured orientation - not estimated yet -> null
 *   roll_sp,pitch_sp  control setpoints - no controller yet     -> null */
static int row_angles(char *b, size_t n, const sample_t *s)
{
    return snprintf(b, n, "[%.3f,null,null,null,null]", (double)s->t_us / 1e6);
}

/* motors: [ t, velL, velR, velL_sp, velR_sp, mL, mR ]
 *   velL,velR         measured wheel speed, rad/s
 *   velL_sp,velR_sp   speed setpoints - no controller yet -> null
 *   mL,mR             commanded motor effort, -1..+1 */
static int row_motors(char *b, size_t n, const sample_t *s)
{
    return snprintf(b, n, "[%.3f,%.3f,%.3f,null,null,%.2f,%.2f]",
        (double)s->t_us / 1e6, s->velL, s->velR, s->mL, s->mR);
}

typedef struct { const char *name; row_fn row; } topic_t;

static const topic_t TOPICS[] = {
    { "imu",    row_imu },
    { "angles", row_angles },
    { "motors", row_motors },
};
#define NUM_TOPICS ((int)(sizeof(TOPICS) / sizeof(TOPICS[0])))

int telemetry_topic_count(void) { return NUM_TOPICS; }

const char *telemetry_topic_name(int topic)
{
    return (topic >= 0 && topic < NUM_TOPICS) ? TOPICS[topic].name : "";
}

int telemetry_topic_json(char *buf, size_t len, int topic,
                         const sample_t *samples, int n)
{
    if (topic < 0 || topic >= NUM_TOPICS) return -1;

    int off = snprintf(buf, len,
        "{\"type\":\"telemetry_batch\",\"topic\":\"%s\",\"rows\":[",
        TOPICS[topic].name);
    if (off < 0 || (size_t)off >= len) return -1;

    for (int i = 0; i < n; i++) {
        if (i) {
            if ((size_t)(off + 1) >= len) return -1;
            buf[off++] = ',';
        }
        int w = TOPICS[topic].row(buf + off, len - off, &samples[i]);
        if (w < 0 || (size_t)(off + w) >= len) return -1;   /* would overflow */
        off += w;
    }

    int w = snprintf(buf + off, len - off, "]}");
    if (w < 0 || (size_t)(off + w) >= len) return -1;
    return off + w;
}
