/**
 * @file balance_pid.h
 * @brief Balance controller: PID on body tilt (outer loop of the cascade).
 *
 * Reads the estimated tilt theta (rad, 0 = upright) and rate theta_dot (rad/s,
 * gyro) and outputs a common wheel-speed command that both inner loops track;
 * driving the wheels under a lean catches the fall. Control law:
 *
 *     e        = theta - theta_ref            (theta_ref: upright trim, ~0)
 *     w_common = Kp*e + Ki*Int(e) + Kd*theta_dot
 *
 * Saturated to +/- BALANCE_W_MAX with conditional-integration anti-windup; D uses
 * the gyro rate directly. Gains are seeds - tune live via balance_pid_set_gains().
 * step()/reset() run in the control loop; gain/setpoint setters (volatile) may be
 * called from the web task.
 */
#pragma once

#include <stdbool.h>

/* Output ceiling for the common wheel-speed command [rad/s], below the wheel's
 * no-load top speed so the inner loop can still track it. */
#define BALANCE_W_MAX      20.0f

/* Tilt magnitude [rad] past which the robot is considered fallen (~34 deg): the
 * supervisor (control.c) then cuts the motors instead of a doomed lunge. */
#define BALANCE_MAX_TILT   0.6f

/* Clear the integrator + internal state; call whenever the loop is (re)enabled. */
void balance_pid_reset(void);

/* Upright trim: the tilt setpoint theta_ref [rad], ~0. A small offset compensates
 * a CoM not exactly over the axle. */
void  balance_pid_set_setpoint(float theta_ref);
float balance_pid_setpoint(void);

/* Runtime PID gains, for on-hardware tuning. Kp [rad/s per rad], Ki [rad/s per
 * rad*s], Kd [rad/s per rad/s]. */
void  balance_pid_set_gains(float kp, float ki, float kd);
float balance_pid_kp(void);
float balance_pid_ki(void);
float balance_pid_kd(void);

/* Restore the compiled-in default gains + zero trim. */
void balance_pid_reset_gains(void);

/* One control step from tilt @p theta (rad), tilt rate @p theta_dot (rad/s) and
 * period @p dt (s): returns the common wheel-speed command (rad/s), saturated to
 * +/- BALANCE_W_MAX, for both wheel_pi setpoints. */
float balance_pid_step(float theta, float theta_dot, float dt);

/* Last raw PID output (before saturation), for analysis/telemetry. */
float balance_pid_raw(void);
