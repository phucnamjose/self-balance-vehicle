/**
 * @file balance_pid.h
 * @brief Balance (standing) controller: a PID on the body tilt (outer loop).
 *
 * The outer loop of the cascade. It reads the estimated tilt @p theta (rad, 0 =
 * upright) and tilt rate @p theta_dot (rad/s, the gyro), and outputs a COMMON
 * wheel-speed command w_common (rad/s) that the mixer feeds to both inner
 * wheel-speed loops (wheel_pi). Driving the wheels forward under a forward lean
 * catches the fall - the classic "move the base under the centre of mass".
 *
 * Control law (positive w_common = forward, for a positive/forward tilt):
 *
 *     e        = theta - theta_ref            (theta_ref: upright trim, ~0)
 *     w_common = Kp*e + Ki*Int(e) + Kd*theta_dot
 *
 * with output saturation to +/- BALANCE_W_MAX and conditional-integration
 * anti-windup (mirrors wheel_pi.c). The D term uses the gyro rate directly - it
 * is already a clean rate, no need to differentiate the noisier angle estimate
 * (docs/theory/angle-estimation.md).
 *
 * Design + math: docs/theory/balance-controller.md and docs/theory/inverted-pendulum.md.
 * The compiled-in gains are SEEDS to be tuned on hardware (the outer loop's
 * gains, unlike the inner loop's, are not identifiable from a bench test);
 * override them live with balance_pid_set_gains().
 *
 * Single instance (one body). Not thread-safe: step()/reset() run in the control
 * loop; the gain/setpoint getters/setters may be called from the web task (those
 * fields are volatile).
 */
#pragma once

#include <stdbool.h>

/* Output ceiling for the common wheel-speed command [rad/s]. Kept below the
 * wheel's no-load top speed so the inner loop can still track it. */
#define BALANCE_W_MAX      30.0f

/* Tilt magnitude [rad] beyond which the robot is considered fallen: past this
 * the small-angle "catch it" strategy cannot recover, so the supervisor
 * (control.c) cuts the motors instead of commanding a doomed full-speed lunge.
 * ~0.6 rad = ~34 deg. */
#define BALANCE_MAX_TILT   0.6f

/* Clear the integrator + internal state. Call whenever the loop is (re)enabled
 * so it starts from a clean slate rather than a stale wound-up integral. */
void balance_pid_reset(void);

/* Upright trim: the tilt setpoint theta_ref [rad] the loop drives toward. Almost
 * always ~0; a small offset compensates a CoM that is not exactly over the axle. */
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

/* One control step: given the estimated tilt @p theta (rad, 0 = upright), the
 * tilt rate @p theta_dot (rad/s, from the gyro), and the tick period @p dt (s),
 * return the common wheel-speed command w_common (rad/s, + = forward), saturated
 * to +/- BALANCE_W_MAX. The caller feeds it to both wheel_pi setpoints. */
float balance_pid_step(float theta, float theta_dot, float dt);

/* Last raw PID output (before saturation), for analysis/telemetry. */
float balance_pid_raw(void);
