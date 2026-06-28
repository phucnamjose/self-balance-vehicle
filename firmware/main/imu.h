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

/* Read one accel/temp/gyro sample in physical units (imu_t.ok=false on failure). */
imu_t mpu6050_read(void);
