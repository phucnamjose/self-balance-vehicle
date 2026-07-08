#include "estimator.h"

#include <math.h>

/* Complementary-filter weight. tau = alpha*dt/(1-alpha); at 200 Hz, alpha=0.98
 * -> tau~0.245 s, crossover ~0.65 Hz - the gyro carries the fast tilt while the
 * accelerometer trims slow drift (docs/theory/angle-estimation.md). Runtime-
 * tunable via estimator_set_alpha(); clamped to [0, COMP_ALPHA_MAX) so the
 * filter stays stable (alpha must stay < 1 to converge). */
#define COMP_ALPHA_DEFAULT  0.98f
#define COMP_ALPHA_MAX      0.9999f
#define DEG2RAD             (float)(M_PI / 180.0)

/* alpha is written by the web task (core 0) and read by the control loop
 * (core 1); volatile keeps the read honest across cores. */
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
        /* Absolute tilt from gravity (see header for the axis mapping). Sign
         * convention: +pitch = tipped forward (+X), +roll = leaned toward -Y, so
         * the balance loop's error e = theta - theta_ref is positive on a forward
         * lean (balance_pid.c) and drives the wheels forward to catch the fall.
         *
         * Pitch carries the leading minus: a forward tilt (rotation about +Y)
         * projects gravity onto -X, so raw atan2(ax, az) reads negative; negating
         * ax flips it to forward-positive AND makes it consistent with the +gy
         * prediction below (d/dt atan2(-ax, az) = +gy). Roll needs no minus: a
         * rotation about +X projects gravity onto +Y, so atan2(ay, az) is already
         * consistent with the +gx prediction. */
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
    /* Publish the current estimate (a failed read this tick just holds the last
     * value); stays unset only until the first good sample seeds the filter. */
    if (s_init) { *roll = s_roll; *pitch = s_pitch; }
    return s_init;
}
