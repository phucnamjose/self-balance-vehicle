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

/* Experiment presets for bring-up / research. Selecting one sets the two feature
 * flags below to a known combination; you can also flip either flag on its own
 * for a custom combo. These only change what the control loop *computes* - they
 * are orthogonal to the STOP/START lifecycle above. */
typedef enum {
    TEST_MOTORS,             /* open-loop motor test: estimation OFF, controller OFF */
    TEST_MOTOR_CONTROLLERS,  /* wheel/motor controller ON, estimation OFF */
    TEST_ANGLES_ESTIMATION,  /* angle estimation ON, controller OFF */
} experiment_mode_t;

/* Apply an experiment preset (overwrites both feature flags). */
void control_set_experiment(experiment_mode_t mode);

/* Individual feature flags read every tick by the control loop. */
bool control_estimation_enabled(void);
void control_set_estimation(bool on);
bool control_controller_enabled(void);
void control_set_controller(bool on);
