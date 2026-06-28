/**
 * @file control.h
 * @brief Hard real-time control loop (core 1) and telemetry reporter (core 0).
 *
 * A GPTimer ISR wakes control_task at CONTROL_HZ; it reads the IMU, drives the
 * motors and, once per second, hands a snapshot to reporter_task via a 1-slot
 * mailbox. reporter_task caches it, logs it and streams it to WebSocket clients.
 */
#pragma once

/* Create the telemetry mailbox, the control + reporter tasks, and start the
 * GPTimer. Call once at boot after the peripherals and web server are up. */
void control_start(void);
