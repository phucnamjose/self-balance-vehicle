/**
 * @file imu.h
 * @brief I2C bus bring-up and the MPU6050 6-axis IMU.
 *
 * Wiring (GY-521 module has its own pull-ups):
 *   VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22, AD0 -> GND (addr 0x68).
 */
#pragma once

#include <stdbool.h>
#include "telemetry.h"   /* imu_t */

/* Create the I2C master bus (MPU6050 lives here). Call once at boot. */
void i2c_init(void);

/* Probe every 7-bit address and log which ones acknowledge. */
void i2c_scan(void);

/* Attach + wake the MPU6050 into a known configuration. Returns true on success. */
bool mpu6050_init(void);

/* Read one accel/temp/gyro sample in physical units (imu_t.ok=false on failure).
 * The stored gyro zero-rate bias (see below) is subtracted from each axis. */
imu_t mpu6050_read(void);

/* Gyro zero-rate bias (deg/s), subtracted from every read so the integrated
 * angle does not drift. Measured with the robot held still - see the gyrocal
 * command (control.c) which averages the gyro at rest and calls the setter.
 * Set/read from the control task; a plain assignment of three floats is benign
 * across the (single) reader. */
void imu_set_gyro_bias(float bx, float by, float bz);
void imu_get_gyro_bias(float *bx, float *by, float *bz);
