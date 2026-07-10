#include "wheel_pi.h"

#include <math.h>
#include <stdbool.h>

/* Per-wheel PI gains, tuned on hardware by pole-zero cancellation (Ti = Kp/Ki
 * parked on the motor's tau). L: Ti = 0.216 s, R: Ti = 0.188 s. */
#define WHEEL_PI_KP_L   0.1455f
#define WHEEL_PI_KI_L   0.6737f
#define WHEEL_PI_KP_R   0.1554f
#define WHEEL_PI_KI_R   0.8265f

/* Per-wheel steady-state gain K [rad/s per duty] from motor identification. Feeds
 * the feedforward u_ff = w_set / K, which supplies most of the hold duty. */
#define WHEEL_PI_K_L    34.36f
#define WHEEL_PI_K_R    32.18f

/* Deadband compensation: a small neutral zone maps to 0 (no dithering at rest);
 * outside it the magnitude is floored so the command clears the motor's start duty. */
#define WHEEL_PI_NEUTRAL      0.02f    /* |u| below this -> 0 */
#define WHEEL_PI_DB_FLOOR     0.1f    /* minimum magnitude once out of neutral */

/* Output duty ceiling below 1.0: leaves H-bridge headroom and room for anti-windup
 * to react before the PWM rails. */
#define WHEEL_PI_DUTY_MAX     0.95f

/* Braking cap when stopping (w_set == 0) while still rolling: limit reverse effort
 * so we decelerate instead of slamming into hard reverse. */
#define WHEEL_PI_BRAKE_MAX    0.4f

/* Integral-state clamp (duty units) at the output limit: the feedforward holds most
 * of the steady-state duty, but this lets the integrator still recover a far-off wheel. */
#define WHEEL_PI_I_MAX        WHEEL_PI_DUTY_MAX

/* First-order low-pass on the quantization-noisy speed measurement; matches the
 * tau_f used in the margin analysis. */
#define WHEEL_PI_TAU_F        0.02f

/* Gains + setpoint: written by the web task (core 0), read by the control loop
 * (core 1); volatile for honest cross-core reads. Per-wheel since the motors differ. */
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

/* Remap a raw command through the deadband: neutral zone -> 0, else floor the
 * magnitude at WHEEL_PI_DB_FLOOR so a small PI output still turns the motor. */
static float deadband_compensate(float u)
{
    if (fabsf(u) < WHEEL_PI_NEUTRAL) return 0.0f;
    if (u > 0.0f) return fmaxf(u,  WHEEL_PI_DB_FLOOR);
    return fminf(u, -WHEEL_PI_DB_FLOOR);
}

float wheel_pi_step(int i, float w_meas, float dt)
{
    /* Seed the filter on the first tick so it doesn't ramp from zero (a fake transient). */
    if (!s_primed[i]) {
        s_w_filt[i] = w_meas;
        s_primed[i] = true;
    }
    float alpha = dt / (WHEEL_PI_TAU_F + dt);
    s_w_filt[i] += alpha * (w_meas - s_w_filt[i]);

    float w_set = s_w_set[i];
    float e     = w_set - s_w_filt[i];

    /* Model-based feedforward: duty to hold w_set in steady state (u_ff = w_set / K),
     * leaving the PI to trim the residual. Guard against a zero/invalid K. */
    float u_ff = 0.0f;
    if (s_ff_en && s_kff[i] != 0.0f) u_ff = w_set / s_kff[i];

    /* FF + P + I, before deadband and saturation. */
    float u = u_ff + s_kp[i] * e + s_i_acc[i];
    s_u_raw[i] = u;                 /* publish the raw command for analysis telemetry */

    float duty     = s_db_en ? deadband_compensate(u) : u;
    float duty_sat = duty;
    if (duty_sat >  WHEEL_PI_DUTY_MAX) duty_sat =  WHEEL_PI_DUTY_MAX;
    if (duty_sat < -WHEEL_PI_DUTY_MAX) duty_sat = -WHEEL_PI_DUTY_MAX;

    /* Safety: when stopping (w_set == 0) but still rolling, cap braking effort so we
     * coast down instead of slamming into reverse. Applied to duty_sat for anti-windup. */
    // if (w_set == 0.0f) {
    //     if (s_w_filt[i] > 0.0f)      duty_sat = fmaxf(duty_sat, -WHEEL_PI_BRAKE_MAX);
    //     else if (s_w_filt[i] < 0.0f) duty_sat = fminf(duty_sat,  WHEEL_PI_BRAKE_MAX);
    // }

    /* Conditional-integration anti-windup: don't wind further into the saturating
     * direction, but allow unwinding. Clamp the state as a second line of defence. */
    bool pushing_high = (duty > duty_sat) && (e > 0.0f);
    bool pushing_low  = (duty < duty_sat) && (e < 0.0f);
    if (!pushing_high && !pushing_low) {
        s_i_acc[i] += s_ki[i] * e * dt;
        if (s_i_acc[i] >  WHEEL_PI_I_MAX) s_i_acc[i] =  WHEEL_PI_I_MAX;
        if (s_i_acc[i] < -WHEEL_PI_I_MAX) s_i_acc[i] = -WHEEL_PI_I_MAX;
    }

    return duty_sat;
}
