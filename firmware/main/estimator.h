/**
 * @file estimator.h
 * @brief Tilt estimator: complementary filter fusing the accel angle with the
 *        integrated gyro rate.
 *
 * Produces body roll/pitch (rad) for the balance loop by blending the two poor raw
 * signals (noisy-but-absolute accel, smooth-but-drifting gyro): gyro high, accel
 * low. Conventions: pitch (about Y) = atan2(-ax, az), rate gy; roll (about X) =
 * atan2(ay, az), rate gx. The minus on ax makes +pitch = tipped forward.
 * update() runs in the control loop; the alpha getter/setter (volatile) may be
 * called from the web task.
 */
#pragma once

#include <stdbool.h>
#include "telemetry.h"   /* imu_t */

/* Reset the filter so the next update() re-seeds from the accelerometer. Call
 * whenever estimation is (re)enabled so it never carries a stale angle. */
void estimator_reset(void);

/* Filter weight (0..1; higher trusts the gyro more). tau = alpha*dt/(1-alpha); the
 * setter clamps to a stable range (< 1). Runtime-tunable during bring-up. */
void  estimator_set_alpha(float a);
float estimator_alpha(void);

/* One step from IMU sample @p imu over period @p dt (s): seeds from the accel on
 * the first good sample, then blends gyro + accel (holds the last estimate on a
 * failed read). Writes @p roll / @p pitch (rad); returns true once valid. */
bool estimator_update(imu_t imu, float dt, float *roll, float *pitch);
