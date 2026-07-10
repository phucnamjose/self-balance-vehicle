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

/* Per-wheel angular-speed setpoint (i = 0 L, 1 R), rad/s (+ = forward). Written
 * by the web terminal, read by the control loop. */
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

/* Restore both wheels' gains (Kp, Ki, K) to the compiled-in defaults. */
void wheel_pi_reset_gains(void);

/* Enable/disable deadband compensation (neutral zone + magnitude floor). When
 * off, the raw PI output is used directly (still saturated). Off by default. */
void wheel_pi_set_deadband(bool on);
bool wheel_pi_deadband(void);

/* Enable/disable the model-based feedforward (u_ff = w_set / K). On by default. */
void wheel_pi_set_ff(bool on);
bool wheel_pi_ff(void);

/* One control step for wheel @p i from measured speed @p w_meas (rad/s) and period
 * @p dt (s): returns the duty in [-1, +1] for motor_set(), using the setpoint above. */
float wheel_pi_step(int i, float w_meas, float dt);

/* Last raw PI output for wheel @p i (before deadband/saturation), for telemetry. */
float wheel_pi_raw(int i);
