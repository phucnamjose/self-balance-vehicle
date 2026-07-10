#include "estimator.h"

#include <math.h>

/* Complementary-filter weight. tau = alpha*dt/(1-alpha), so alpha is rate-dependent:
 * 0.992 keeps tau ~= 0.245 s (crossover ~0.65 Hz) at 500 Hz. The gyro carries fast
 * tilt, the accel trims slow drift. Runtime-tunable; clamped < 1 to stay stable. */
#define COMP_ALPHA_DEFAULT  0.992f
#define COMP_ALPHA_MAX      0.9999f
#define DEG2RAD             (float)(M_PI / 180.0)

/* alpha: written by the web task (core 0), read by the control loop (core 1);
 * volatile keeps cross-core reads honest. */
static volatile float s_alpha = COMP_ALPHA_DEFAULT;

/* Filter state, touched only by the control loop via update(). */
static float s_roll, s_pitch;   /* current estimate, rad */
static bool  s_init;            /* seeded from the accelerometer yet? */

void estimator_reset(void) { s_init = false; }

float estimator_alpha(void) { return s_alpha; }

void estimator_set_alpha(float a)
{
    if (a < 0.0f)           a = 0.0f;
    if (a > COMP_ALPHA_MAX) a = COMP_ALPHA_MAX;
    s_alpha = a;
}

bool estimator_update(imu_t imu, float dt, float *roll, float *pitch)
{
    if (imu.ok) {
        /* Absolute tilt from gravity. Sign convention: +pitch = tipped forward,
         * +roll = leaned toward -Y, so the balance error is positive on a forward
         * lean. Pitch negates ax so a forward tilt reads positive and matches the
         * +gy prediction; roll needs no minus (atan2(ay, az) matches +gx). */
        float pitch_acc = atan2f(-imu.ax, imu.az);
        float roll_acc  = atan2f(imu.ay, imu.az);
        if (!s_init) {
            s_pitch = pitch_acc;   /* seed at the accelerometer angle */
            s_roll  = roll_acc;
            s_init  = true;
        } else {
            /* Predict with the gyro (integrate the rate), correct toward accel. */
            float a = s_alpha;
            s_pitch = a * (s_pitch + imu.gy * DEG2RAD * dt) + (1.0f - a) * pitch_acc;
            s_roll  = a * (s_roll  + imu.gx * DEG2RAD * dt) + (1.0f - a) * roll_acc;
        }
    }
    /* Publish the current estimate (a failed read holds the last value); unset only
     * until the first good sample seeds the filter. */
    if (s_init) { *roll = s_roll; *pitch = s_pitch; }
    return s_init;
}
