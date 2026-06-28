/**
 * @file encoders.h
 * @brief Quadrature wheel encoders via the PCNT hardware counter (index 0=L, 1=R).
 *
 * PCNT counts in hardware (16-bit); on each rollover a watch callback folds the
 * limit into a 64-bit accumulator, giving a full position.
 */
#pragma once

#include <stdint.h>

/* Configure both PCNT units for 4x quadrature decoding. Call once at boot. */
void encoder_init(void);

/* Total position for wheel @p i (accumulator + current hardware count). */
int64_t encoder_count(int i);

/* Reset wheel @p i position to 0. */
void encoder_reset(int i);
