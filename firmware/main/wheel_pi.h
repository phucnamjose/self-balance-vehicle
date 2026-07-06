/**
 * @file wheel_pi.h
 * @brief Per-wheel angular-speed PI controller (inner loop of the cascade).
 *
 * One instance per wheel (index 0 = L, 1 = R) regulates that wheel's angular
 * speed to a common setpoint, so the two non-identical motors behave the same to
 * whatever sits above them. Each step adds a model-based feedforward
 * (u_ff = w_set / K) to PI on the speed error, then applies deadband
 * compensation and output saturation, with conditional-integration anti-windup.
 *
 * Design + math: docs/theory/wheel-speed-controller.md and docs/theory/pi-tuning.md.
 * Gains are per-wheel (the two motors are not identical) and default to the
 * values tuned on hardware; override at runtime with wheel_pi_set_gains().
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

/* One control step for wheel @p i (0 = L, 1 = R): given the measured wheel speed
 * @p w_meas (rad/s) and the tick period @p dt (s), return the duty in [-1, +1]
 * to hand to motor_set(). Uses the common setpoint set above. */
float wheel_pi_step(int i, float w_meas, float dt);

/* Last raw PI output for wheel @p i (Kp*e + integral), i.e. the command BEFORE
 * deadband compensation and saturation. For analysis/telemetry only. */
float wheel_pi_raw(int i);
