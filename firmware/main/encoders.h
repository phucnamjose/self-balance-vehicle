/**
 * @file encoders.h
 * @brief Quadrature wheel encoders via the PCNT hardware counter (index 0=L, 1=R).
 *
 * PCNT counts in hardware (16-bit); on each rollover a watch callback folds the
 * limit into a 64-bit accumulator, giving a full position.
 */
#pragma once

#include <stdint.h>

/* GB37-520 with 4x quadrature decoding: 11 PPR * 30:1 gearbox * 4 = 1320 counts
 * per full wheel (output-shaft) revolution. This is the number that maps raw
 * counts to physical angle/distance. */
#define ENC_COUNTS_PER_WHEEL_REV   1320

/* Configure both PCNT units for 4x quadrature decoding. Call once at boot. */
void encoder_init(void);

/* Total position for wheel @p i (accumulator + current hardware count). */
int64_t encoder_count(int i);

/* Reset wheel @p i position to 0. */
void encoder_reset(int i);

/* Continuous (unwrapped) wheel angle for wheel @p i, in radians. Positive =
 * forward. Grows without bound with position; double keeps full precision. */
double encoder_angle_rad(int i);

/* Convert a counts/second rate (e.g. telemetry velL/velR) to wheel angular
 * speed in radians/second. */
float encoder_cps_to_radps(int32_t counts_per_sec);
