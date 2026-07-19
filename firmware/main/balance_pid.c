#include "balance_pid.h"

#include <math.h>

/* Default PID gains (seeds - tune on hardware). The outer loop outputs a wheel-speed
 * command, so gains are in rad/s-of-speed per rad tilt (Kp), per rad*s (Ki), per
 * rad/s (Kd). In this velocity cascade Ki is the restoring stiffness (must clear
 * g/r ~= 302 to beat gravity; 450 gives ~50% margin), Kp is damping, and Kd behaves
 * like added base inertia - so keep it SMALL. Seeds are the flat optimum from
 * simulation/tune_cascade.m (multi-rate 500/250/250, inner tau_f=0). Bring-up: with
 * the fast tau_f=0 inner loop, hot gains (esp. Kd) chatter the duty and the cart
 * accel then corrupts the tilt estimate - raise gently. */
#define BALANCE_KP_DEFAULT   45.00f
#define BALANCE_KI_DEFAULT   450.00f
#define BALANCE_KD_DEFAULT   3.00f

/* Integral-state clamp [rad/s], at the output ceiling: a persistent lean can call
 * on full drive authority but no more. */
#define BALANCE_I_MAX        BALANCE_W_MAX

/* Gains + setpoint: written by the web task (core 0), read by the control loop
 * (core 1); volatile keeps cross-core reads honest. */
static volatile float s_kp        = BALANCE_KP_DEFAULT;
static volatile float s_ki        = BALANCE_KI_DEFAULT;
static volatile float s_kd        = BALANCE_KD_DEFAULT;
static volatile float s_theta_ref = 0.0f;

/* State, touched only by the control loop via step()/reset(). */
static float s_i_acc;    /* integral accumulator, rad/s */
static float s_u_raw;    /* last raw output (pre-saturation), for telemetry */

void balance_pid_reset(void)
{
    s_i_acc = 0.0f;
    s_u_raw = 0.0f;
}

void  balance_pid_set_setpoint(float theta_ref) { s_theta_ref = theta_ref; }
float balance_pid_setpoint(void)                { return s_theta_ref; }

void  balance_pid_set_gains(float kp, float ki, float kd) { s_kp = kp; s_ki = ki; s_kd = kd; }
float balance_pid_kp(void) { return s_kp; }
float balance_pid_ki(void) { return s_ki; }
float balance_pid_kd(void) { return s_kd; }

void balance_pid_reset_gains(void)
{
    s_kp = BALANCE_KP_DEFAULT;
    s_ki = BALANCE_KI_DEFAULT;
    s_kd = BALANCE_KD_DEFAULT;
    s_theta_ref = 0.0f;
}

float balance_pid_raw(void) { return s_u_raw; }

float balance_pid_step(float theta, float theta_dot, float dt)
{
    float e = theta - s_theta_ref;   /* + for a forward lean */

    /* P + I + D. D uses the gyro rate directly (already a clean rate). */
    float u = s_kp * e + s_i_acc + s_kd * theta_dot;
    s_u_raw = u;

    float u_sat = u;
    if (u_sat >  BALANCE_W_MAX) u_sat =  BALANCE_W_MAX;
    if (u_sat < -BALANCE_W_MAX) u_sat = -BALANCE_W_MAX;

    /* Conditional-integration anti-windup: don't wind further into the saturating
     * direction, but allow unwinding. Clamp the state as a second line of defence. */
    bool pushing_high = (u > u_sat) && (e > 0.0f);
    bool pushing_low  = (u < u_sat) && (e < 0.0f);
    if (!pushing_high && !pushing_low) {
        s_i_acc += s_ki * e * dt;
        if (s_i_acc >  BALANCE_I_MAX) s_i_acc =  BALANCE_I_MAX;
        if (s_i_acc < -BALANCE_I_MAX) s_i_acc = -BALANCE_I_MAX;
    }

    return u_sat;
}
