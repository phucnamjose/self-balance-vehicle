#include "estimator.h"

#include <math.h>

/* Complementary-filter weight, quoted as alpha at COMP_REF_DT. The filter turns it into
 * a time constant tau = alpha*dt_ref/(1-alpha) and recomputes the weight for the actual
 * dt, so the ~0.245 s crossover holds if the loop rate changes. Clamped < 1 to stay
 * stable; runtime-tunable via the web UI. */
#define COMP_ALPHA_DEFAULT  0.992f
#define COMP_ALPHA_MAX      0.9999f
#define COMP_REF_DT         (1.0f / 500.0f)   // rate alpha is quoted at [s]
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
            /* Gyro predict + accel correct. Rescale a_ref to the actual dt (via tau) so
             * the crossover is rate-independent; at dt = dt_ref this gives a = a_ref. */
            float a_ref = s_alpha;
            float tau   = a_ref * COMP_REF_DT / (1.0f - a_ref);
            float a     = tau / (tau + dt);
            s_pitch = a * (s_pitch + imu.gy * DEG2RAD * dt) + (1.0f - a) * pitch_acc;
            s_roll  = a * (s_roll  + imu.gx * DEG2RAD * dt) + (1.0f - a) * roll_acc;
        }
    }
    /* Publish the current estimate (a failed read holds the last value); unset only
     * until the first good sample seeds the filter. */
    if (s_init) { *roll = s_roll; *pitch = s_pitch; }
    return s_init;
}
