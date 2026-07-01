#include "telemetry.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
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
 * Each topic is a projection of sample_t packed into a compact binary frame:
 *
 *   [u8 topic_id][u8 field_count][u16 sample_count]   (little-endian)
 *   then, per sample: uint32 t_ms, followed by (field_count-1) float32 values
 *
 * All multi-byte values are little-endian (matches the ESP32 and the browser's
 * DataView with littleEndian=true). Column 0 is always the timestamp in
 * milliseconds since boot; the rest are float32. Fields not produced yet
 * (orientation estimate, control setpoints) are packed as NaN, which the client
 * renders as empty CSV cells. The per-topic column order below is the contract
 * the client decoder must mirror. */
static inline uint8_t *put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;  p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);  p[3] = (uint8_t)(v >> 24);
    return p + 4;
}

static inline uint8_t *put_f32(uint8_t *p, float f)
{
    uint32_t v;
    memcpy(&v, &f, sizeof(v));
    return put_u32(p, v);
}

static inline uint8_t *put_t(uint8_t *p, const sample_t *s)
{
    return put_u32(p, (uint32_t)(s->t_us / 1000));   /* ms since boot */
}

typedef int (*pack_fn)(uint8_t *buf, const sample_t *s);   /* returns bytes */

/* imu: [ t(ms), ax, ay, az, gx, gy, gz, temp, ok ]
 *   ax,ay,az  g   gx,gy,gz  deg/s   temp  degC   ok  1.0/0.0 */
static int pack_imu(uint8_t *b, const sample_t *s)
{
    uint8_t *p = put_t(b, s);
    p = put_f32(p, s->imu.ax); p = put_f32(p, s->imu.ay); p = put_f32(p, s->imu.az);
    p = put_f32(p, s->imu.gx); p = put_f32(p, s->imu.gy); p = put_f32(p, s->imu.gz);
    p = put_f32(p, s->imu.temp_c);
    p = put_f32(p, s->imu.ok ? 1.0f : 0.0f);
    return (int)(p - b);
}

/* angles: [ t(ms), roll, pitch, roll_sp, pitch_sp ]  (radians)
 *   all NaN until orientation estimation / a controller exist */
static int pack_angles(uint8_t *b, const sample_t *s)
{
    uint8_t *p = put_t(b, s);
    p = put_f32(p, NAN); p = put_f32(p, NAN);
    p = put_f32(p, NAN); p = put_f32(p, NAN);
    return (int)(p - b);
}

/* motors: [ t(ms), velL, velR, velL_sp, velR_sp, mL, mR ]
 *   velL,velR  rad/s (measured)   *_sp  NaN (no controller)   mL,mR  -1..+1 */
static int pack_motors(uint8_t *b, const sample_t *s)
{
    uint8_t *p = put_t(b, s);
    p = put_f32(p, s->velL); p = put_f32(p, s->velR);
    p = put_f32(p, NAN);     p = put_f32(p, NAN);
    p = put_f32(p, s->mL);   p = put_f32(p, s->mR);
    return (int)(p - b);
}

typedef struct { const char *name; uint8_t fields; pack_fn pack; } topic_t;

static const topic_t TOPICS[] = {
    { "imu",    9, pack_imu },
    { "angles", 5, pack_angles },
    { "motors", 7, pack_motors },
};
#define NUM_TOPICS ((int)(sizeof(TOPICS) / sizeof(TOPICS[0])))

int telemetry_topic_count(void) { return NUM_TOPICS; }

const char *telemetry_topic_name(int topic)
{
    return (topic >= 0 && topic < NUM_TOPICS) ? TOPICS[topic].name : "";
}

int telemetry_topic_pack(uint8_t *buf, size_t len, int topic,
                         const sample_t *samples, int n)
{
    if (topic < 0 || topic >= NUM_TOPICS) return -1;

    uint8_t fields = TOPICS[topic].fields;
    size_t need = 4 + (size_t)n * fields * 4;   /* header + n * fields * 4B */
    if (need > len) return -1;

    buf[0] = (uint8_t)topic;
    buf[1] = fields;
    buf[2] = (uint8_t)(n & 0xff);
    buf[3] = (uint8_t)((n >> 8) & 0xff);

    uint8_t *p = buf + 4;
    for (int i = 0; i < n; i++) {
        p += TOPICS[topic].pack(p, &samples[i]);
    }
    return (int)(p - buf);
}
