/**
 * @file estimator.h
 * @brief Orientation (tilt) estimator: a complementary filter fusing the IMU's
 *        accelerometer angle with the integrated gyro rate.
 *
 * Produces the body roll/pitch (rad) the balance loop needs. The two raw signals
 * are individually poor - the accelerometer angle is absolute but noisy and
 * corrupted by the robot's own motion, the integrated gyro is smooth but drifts -
 * so we blend them: gyro at high frequency, accelerometer at low. Conventions
 * (docs/hardware/robot-mechanics.md): pitch (the balance tilt theta) is rotation
 * about Y, theta_acc = atan2(-ax, az) with rate gy; roll is about X,
 * roll_acc = atan2(ay, az) with rate gx. The minus on ax makes +pitch = tipped
 * forward, so the balance loop's error is positive on a forward lean; it also
 * makes the accel term agree with the +gy prediction. Filter theory + the
 * alpha/tau relationship: docs/theory/angle-estimation.md.
 *
 * Single instance (one IMU). Not thread-safe: update() runs in the control loop;
 * the alpha getter/setter may be called from the web task (alpha is volatile).
 */
#pragma once

#include <stdbool.h>
#include "telemetry.h"   /* imu_t */

/* Reset the filter so the next update() re-seeds from the accelerometer. Call
 * whenever estimation is (re)enabled so it never carries a stale angle. */
void estimator_reset(void);

/* Complementary-filter weight (0..1; higher trusts the gyro more, so smoother and
 * more motion-immune but slower to correct gyro drift). tau = alpha*dt/(1-alpha).
 * The setter clamps to a stable range (< 1). Runtime-tunable during bring-up. */
void  estimator_set_alpha(float a);
float estimator_alpha(void);

/* One estimator step from IMU sample @p imu over tick period @p dt (s). On the
 * first good sample it seeds from the accelerometer; afterwards it blends
 * gyro + accel. Holds the last estimate if this sample's read failed. Writes
 * @p roll / @p pitch (rad) and returns true once a valid estimate exists
 * (false only before the first good sample after a reset). */
bool estimator_update(imu_t imu, float dt, float *roll, float *pitch);
