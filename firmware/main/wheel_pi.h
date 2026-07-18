/**
 * @file wheel_pi.h
 * @brief Per-wheel angular-speed PI controller (inner loop of the cascade).
 *
 * One instance per wheel (0 = L, 1 = R) regulates its speed to a common setpoint
 * so the two non-identical motors behave the same. Each step adds a feedforward
 * (u_ff = w_set / K) to PI on the speed error, then deadband compensation and
 * saturation with conditional-integration anti-windup. Gains are per-wheel,
 * tuned on hardware; override at runtime with wheel_pi_set_gains().
 */
#pragma once

#include <stdbool.h>

/* Clear both wheels' integrator + measurement-filter state. Call whenever the
 * controller is (re)enabled so it starts from a clean slate. */
void wheel_pi_reset(void);

/* Wheel-speed setpoint saturation [rad/s]. The reachable free-run speed is about
 * K*duty_max ~= 32 rad/s, so cap the setpoint just below that to keep it achievable
 * and leave the PI some duty headroom. Enforced in wheel_pi_set_setpoint(), so every
 * source - the 'speed' command, the balance loop, speed playback - is bounded. */
#define WHEEL_PI_WSET_MAX 29.0f

/* Per-wheel angular-speed setpoint (i = 0 L, 1 R), rad/s (+ = forward). Written
 * by the web terminal, read by the control loop; clamped to +/-WHEEL_PI_WSET_MAX. */
void  wheel_pi_set_setpoint(int i, float w_set);
float wheel_pi_setpoint(int i);

/* Per-wheel runtime gains (i = 0 L, 1 R), for on-hardware tuning. */
void  wheel_pi_set_gains(int i, float kp, float ki);
float wheel_pi_kp(int i);
float wheel_pi_ki(int i);

/* Per-wheel feedforward gain K [rad/s per duty] (i = 0 L, 1 R). The feedforward
 * term is u_ff = w_set / K. */
void  wheel_pi_set_kff(int i, float k);
float wheel_pi_kff(int i);

/* Restore both wheels' gains (Kp, Ki, K), setpoint weight and slew rate to the defaults. */
void wheel_pi_reset_gains(void);

/* Setpoint slew-rate limit [rad/s per s], shared by both wheels (0 = off, instant setpoint).
 * The loop tracks a ramped reference instead of the raw step, killing the proportional kick
 * without any steady-state penalty. This is the recommended overshoot fix; set it near the
 * wheel's achievable acceleration (~150-200 rad/s^2). */
void  wheel_pi_set_slew(float rate);
float wheel_pi_slew(void);

/* Speed-measurement low-pass time constant tau_f [s], shared by both wheels (0 = off).
 * Higher smooths the quantization-noisy encoder speed but adds phase lag (less margin,
 * more overshoot, later braking); lower is snappier but noisier. */
void  wheel_pi_set_tau_f(float tau);
float wheel_pi_tau_f(void);

/* Proportional setpoint weight b in [0,1], shared by both wheels (2-DOF PI): P acts on
 * (b*w_ref - w_filt), the integral on the full error. b=1 is a plain PI (recommended).
 * Lowering b also trims the kick but leaves a steady-state term Kp*(1-b)*w_ref that the
 * integrator must cancel; past ~11 rad/s it exceeds the integral clamp, so prefer the
 * slew limit for the full speed range. */
void  wheel_pi_set_bweight(float b);
float wheel_pi_bweight(void);

/* Enable/disable deadband compensation (neutral zone + magnitude floor). When
 * off, the raw PI output is used directly (still saturated). Off by default. */
void wheel_pi_set_deadband(bool on);
bool wheel_pi_deadband(void);

/* Enable/disable the model-based feedforward (u_ff = w_ref / K). Off by default. */
void wheel_pi_set_ff(bool on);
bool wheel_pi_ff(void);

/* One control step for wheel @p i from measured speed @p w_meas (rad/s) and period
 * @p dt (s): returns the duty in [-1, +1] for motor_set(), using the setpoint above. */
float wheel_pi_step(int i, float w_meas, float dt);

/* Last raw PI output for wheel @p i (before deadband/saturation), for telemetry. */
float wheel_pi_raw(int i);
