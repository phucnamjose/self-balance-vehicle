/**
 * @file motors.h
 * @brief XY-160D dual H-bridge motors and the open-loop command state.
 *
 * Each motor: ENx = PWM speed, IN1/IN2 = direction. Index 0 = left, 1 = right.
 * The web terminal sets per-motor commands; the control loop reads them and
 * drives the motors each tick.
 */
#pragma once

/* Configure direction GPIOs and the LEDC PWM channels. Call once at boot. */
void motor_init(void);

/* Apply a signed command to one motor: sign -> direction, magnitude -> PWM duty.
 * cmd is clamped to -1.0..+1.0; cmd 0 brakes/stops. */
void motor_set(int i, float cmd);

/* Open-loop command per motor, -1.0..+1.0 (sign = direction). Shared between the
 * web terminal (writer) and the control loop (reader). */
void  motor_cmd_set(int i, float cmd);
float motor_cmd_get(int i);
