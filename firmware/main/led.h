/**
 * @file led.h
 * @brief Non-blocking notification LEDs (Left=GPIO17, Right=GPIO16, driven together).
 *
 * A tiny task owns the LEDs. Callers post a blink pattern to a 1-slot mailbox and
 * return immediately - no delays in the caller's context. A new request always
 * preempts the one in progress.
 */
#pragma once

#include <stdint.h>

/* Create the LED GPIOs, mailbox and the LED task. Call once at boot. */
void led_init(void);

/* Non-blocking. Examples:
 *   led_blink(60, 60, 3)     -> three quick blinks, then off
 *   led_blink(60, 1940, -1)  -> slow heartbeat forever
 *   led_blink(1, 0, -1)      -> solid on
 *   led_blink(0, 0, 0)       -> off                                            */
void led_blink(uint16_t on_ms, uint16_t off_ms, int16_t count);

void led_off(void);
void led_on(void);
void led_heartbeat(void);       /* slow ~0.5 Hz idle heartbeat */
void led_heartbeat_1hz(void);   /* faster 1 Hz heartbeat (balance loop running) */
