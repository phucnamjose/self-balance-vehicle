/**
 * @file control.h
 * @brief Hard real-time control loop (core 1) and telemetry reporter (core 0).
 *
 * A GPTimer ISR wakes control_task at CONTROL_HZ; it reads the IMU, drives the
 * motors and, once per second, hands a snapshot to reporter_task via a 1-slot
 * mailbox. reporter_task caches it, logs it and streams it to WebSocket clients.
 */
#pragma once

#include <stdbool.h>

/* Control-task lifecycle modes. */
typedef enum {
    STOP_CONTROL,    /* control task fully stopped (deleted, not suspended), motors off */
    START_CONTROL,   /* control task running */
} control_mode_t;

/* Create the telemetry mailbox + reporter task and start the control task
 * (START_CONTROL). Call once at boot after the web server is up. */
void control_start(void);

/* Current lifecycle mode. */
control_mode_t control_mode(void);

/* Switch lifecycle mode:
 *   STOP_CONTROL  - stop the control task (delete it at a safe point, not just
 *                   suspend) and force the motors off. Distinct from the 'stop'
 *                   motor-test command, which only zeroes the outputs while the
 *                   task keeps running. Required before an OTA flash.
 *   START_CONTROL - start or restart the control task.
 * The peripherals and GPTimer are initialised once and reused across restarts. */
void control_set_mode(control_mode_t mode);
