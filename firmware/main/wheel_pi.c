#include "wheel_pi.h"

#include <math.h>
#include <stdbool.h>

/* Per-wheel PI gains, tuned on hardware (pole-zero cancellation from each
 * motor's identified K, tau; see docs/theory/pi-tuning.md). Ti = Kp/Ki is the
 * integral time, parked on the motor's tau:
 *   L: Kp = 0.1455, Ki = 0.6737  (Ti = 0.216 s)
 *   R: Kp = 0.1554, Ki = 0.8265  (Ti = 0.188 s) */
#define WHEEL_PI_KP_L   0.1455f
#define WHEEL_PI_KI_L   0.6737f
#define WHEEL_PI_KP_R   0.1554f
#define WHEEL_PI_KI_R   0.8265f

/* Per-wheel steady-state gain K [rad/s per unit duty] from motor identification
 * (experiments/motors_identify/motor_id.m). Used by the feedforward term
 * u_ff = w_set / K, which supplies the bulk of the hold duty so the integrator
 * only has to trim the residual. */
#define WHEEL_PI_K_L    34.36f
#define WHEEL_PI_K_R    32.18f

/* Deadband compensation: inside a small neutral zone the command is forced to 0
 * (no dithering around rest); outside it, the magnitude is floored so the command
 * clears the duty at which the motor actually starts turning. */
#define WHEEL_PI_NEUTRAL      0.02f    /* |u| below this -> 0 */
#define WHEEL_PI_DB_FLOOR     0.1f    /* minimum magnitude once out of neutral */

/* Output duty ceiling: cap below 1.0 so there is headroom for the H-bridge and
 * so the anti-windup has room to react before the PWM truly rails. */
#define WHEEL_PI_DUTY_MAX     0.95f

/* Braking authority when commanded to stop (w_set == 0) while the wheel is still
 * rolling: limit the reverse effort so we decelerate instead of slamming into
 * hard reverse (which would jerk the robot and stress the gearbox). */
#define WHEEL_PI_BRAKE_MAX    0.4f

/* Integral-state clamp, in duty units. The feedforward supplies most of the
 * steady-state hold duty; the integrator only trims the residual, but clamp it
 * at the output limit so it can still recover a wheel that is far off. */
#define WHEEL_PI_I_MAX        WHEEL_PI_DUTY_MAX

/* First-order low-pass on the (quantization-noisy at 200 Hz) speed measurement.
 * Matches the tau_f used in the Nyquist/Bode margin analysis (pi-tuning.md). */
#define WHEEL_PI_TAU_F        0.02f

/* Gains and setpoint are written from the web task (core 0) and read by the
 * control loop (core 1); volatile keeps the reads honest across cores. Gains are
 * per-wheel because the two motors are not identical. */
static volatile float s_kp[2]    = { WHEEL_PI_KP_L, WHEEL_PI_KP_R };
static volatile float s_ki[2]    = { WHEEL_PI_KI_L, WHEEL_PI_KI_R };
static volatile float s_kff[2]   = { WHEEL_PI_K_L, WHEEL_PI_K_R };   /* feedforward K, rad/s per duty */
static volatile float s_w_set[2] = { 0.0f, 0.0f };   /* per-wheel setpoint, rad/s */
static volatile bool  s_db_en    = false;            /* deadband compensation on/off */
static volatile bool  s_ff_en    = true;             /* feedforward on/off */

/* Per-wheel state, touched only by the control loop. */
static float s_i_acc[2];    /* integral accumulator, duty units */
static float s_w_filt[2];   /* filtered measured speed, rad/s */
static float s_u_raw[2];    /* last raw PI output (pre-deadband/saturation), for telemetry */
static bool  s_primed[2];   /* filter seeded on the first step after a reset */

void wheel_pi_reset(void)
{
    for (int i = 0; i < 2; i++) {
        s_i_acc[i]  = 0.0f;
        s_u_raw[i]  = 0.0f;
        s_w_filt[i] = 0.0f;
        s_primed[i] = false;
    }
}

void  wheel_pi_set_setpoint(int i, float w_set) { s_w_set[i] = w_set; }
float wheel_pi_setpoint(int i)                   { return s_w_set[i]; }

float wheel_pi_raw(int i) { return s_u_raw[i]; }

void  wheel_pi_set_gains(int i, float kp, float ki) { s_kp[i] = kp; s_ki[i] = ki; }
float wheel_pi_kp(int i)                             { return s_kp[i]; }
float wheel_pi_ki(int i)                             { return s_ki[i]; }

void wheel_pi_reset_gains(void)
{
    s_kp[0] = WHEEL_PI_KP_L; s_ki[0] = WHEEL_PI_KI_L; s_kff[0] = WHEEL_PI_K_L;
    s_kp[1] = WHEEL_PI_KP_R; s_ki[1] = WHEEL_PI_KI_R; s_kff[1] = WHEEL_PI_K_R;
}

void  wheel_pi_set_kff(int i, float k) { s_kff[i] = k; }
float wheel_pi_kff(int i)              { return s_kff[i]; }

void wheel_pi_set_deadband(bool on) { s_db_en = on; }
bool wheel_pi_deadband(void)        { return s_db_en; }

void wheel_pi_set_ff(bool on) { s_ff_en = on; }
bool wheel_pi_ff(void)        { return s_ff_en; }

/* Remap a raw command through the deadband: a 2% neutral zone maps to 0, and
 * outside it the magnitude is floored at WHEEL_PI_DB_FLOOR so a small PI output
 * still clears the duty where the motor starts turning. */
static float deadband_compensate(float u)
{
    if (fabsf(u) < WHEEL_PI_NEUTRAL) return 0.0f;
    if (u > 0.0f) return fmaxf(u,  WHEEL_PI_DB_FLOOR);
    return fminf(u, -WHEEL_PI_DB_FLOOR);
}

float wheel_pi_step(int i, float w_meas, float dt)
{
    /* Seed the measurement filter on the first tick so it doesn't ramp up from
     * zero (which would look like a big transient error). */
    if (!s_primed[i]) {
        s_w_filt[i] = w_meas;
        s_primed[i] = true;
    }
    float alpha = dt / (WHEEL_PI_TAU_F + dt);
    s_w_filt[i] += alpha * (w_meas - s_w_filt[i]);

    float w_set = s_w_set[i];
    float e     = w_set - s_w_filt[i];

    /* Model-based feedforward: the duty that the identified plant needs to hold
     * w_set in steady state (u_ff = w_set / K). Leaves the PI to correct only the
     * residual. Guard against a zero/invalid K. */
    float u_ff = 0.0f;
    if (s_ff_en && s_kff[i] != 0.0f) u_ff = w_set / s_kff[i];

    /* FF + P + I, before deadband and saturation. */
    float u = u_ff + s_kp[i] * e + s_i_acc[i];
    s_u_raw[i] = u;                 /* publish the raw command for analysis telemetry */

    float duty     = s_db_en ? deadband_compensate(u) : u;
    float duty_sat = duty;
    if (duty_sat >  WHEEL_PI_DUTY_MAX) duty_sat =  WHEEL_PI_DUTY_MAX;
    if (duty_sat < -WHEEL_PI_DUTY_MAX) duty_sat = -WHEEL_PI_DUTY_MAX;

    /* Safety: when told to stop (w_set == 0) but still rolling, cap the braking
     * effort so we coast down instead of slamming into hard reverse. Applied to
     * duty_sat so the anti-windup below treats it as the effective limit. */
    if (w_set == 0.0f) {
        if (s_w_filt[i] > 0.0f)      duty_sat = fmaxf(duty_sat, -WHEEL_PI_BRAKE_MAX);
        else if (s_w_filt[i] < 0.0f) duty_sat = fminf(duty_sat,  WHEEL_PI_BRAKE_MAX);
    }

    /* Conditional-integration anti-windup: don't wind the integrator further in
     * the direction that is already saturating the output; still allow it to
     * unwind. Clamp the state as a second line of defence. */
    bool pushing_high = (duty > duty_sat) && (e > 0.0f);
    bool pushing_low  = (duty < duty_sat) && (e < 0.0f);
    if (!pushing_high && !pushing_low) {
        s_i_acc[i] += s_ki[i] * e * dt;
        if (s_i_acc[i] >  WHEEL_PI_I_MAX) s_i_acc[i] =  WHEEL_PI_I_MAX;
        if (s_i_acc[i] < -WHEEL_PI_I_MAX) s_i_acc[i] = -WHEEL_PI_I_MAX;
    }

    return duty_sat;
}
