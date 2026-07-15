/**
 * @file imu.h
 * @brief I2C bus bring-up and the MPU6050 6-axis IMU.
 *
 * Wiring (GY-521 module has its own pull-ups):
 *   VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22, AD0 -> GND (addr 0x68).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "telemetry.h"   /* imu_t */

/* Create the I2C master bus (MPU6050 lives here). Call once at boot. */
void i2c_init(void);

/* Probe every 7-bit address and log which ones acknowledge. */
void i2c_scan(void);

/* Attach + wake the MPU6050 into a known configuration (500 Hz, FIFO on, data-ready
 * INT on GPIO34). Returns true on success. Idempotent for retry. */
bool mpu6050_init(void);

/* Read one accel/temp/gyro sample directly from the data registers, in physical units
 * (imu_t.ok=false on failure). The stored gyro zero-rate bias is subtracted. */
imu_t mpu6050_read(void);

/* Read the newest buffered sample: drain the FIFO to its last frame (skipping older
 * ones), or resync via the data registers on overflow/desync. Returns false on I2C
 * failure. Use with mpu6050_dr_count() to size the skip. */
bool mpu6050_read_newest(imu_t *out);

/* Running count of data-ready INT edges (GPIO34), one per MPU sample. Diff it across
 * ticks to get new-sample/skip counts and liveness with no I2C. */
uint32_t mpu6050_dr_count(void);

/* Gyro zero-rate bias (deg/s), subtracted from every read so the angle doesn't
 * drift. Measured at rest by the gyrocal command (control.c). */
void imu_set_gyro_bias(float bx, float by, float bz);
void imu_get_gyro_bias(float *bx, float *by, float *bz);
