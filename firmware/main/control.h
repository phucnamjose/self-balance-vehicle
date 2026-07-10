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

/* Switch lifecycle mode. STOP_CONTROL deletes the task at a safe point and forces
 * the motors off (required before an OTA flash); START_CONTROL (re)starts it.
 * Peripherals and the GPTimer are initialised once and reused across restarts. */
void control_set_mode(control_mode_t mode);

/* Experiment presets for bring-up/research: each sets the feature flags below to a
 * known combination (flip flags individually for a custom combo). These only change
 * what the loop computes, orthogonal to the STOP/START lifecycle. */
typedef enum {
    TEST_MOTORS,             /* open-loop motor test: estimation OFF, controller OFF */
    TEST_MOTOR_CONTROLLERS,  /* wheel/motor controller ON, estimation OFF */
    TEST_ANGLES_ESTIMATION,  /* angle estimation ON, controller OFF */
    TEST_BALANCE,            /* self-balance: estimation + wheel controller + balance loop ON */
} experiment_mode_t;

/* Apply an experiment preset (overwrites the feature flags). */
void control_set_experiment(experiment_mode_t mode);

/* Individual feature flags, read every tick. The balance (outer) loop needs both
 * estimation and the wheel controller enabled to have any effect. */
bool control_estimation_enabled(void);
void control_set_estimation(bool on);
bool control_controller_enabled(void);
void control_set_controller(bool on);
bool control_balance_enabled(void);
void control_set_balance(bool on);

/* Scripted open-loop motor playback (TEST_MOTORS only): a list of steps, each
 * "hold (mL,mR) for dur seconds". Fill incrementally: begin() clears, append()
 * adds a step (returns the new count, -1 full, -2 if dur <= 0), start() plays,
 * stop() halts and parks. active()/len()/pos() report progress. */
void control_playback_begin(void);
int  control_playback_append(float dur, float mL, float mR);
void control_playback_start(void);
void control_playback_stop(void);
bool control_playback_active(void);
int  control_playback_len(void);
int  control_playback_pos(void);

/* Deadband sweep (TEST_MOTORS only): ramps both motors 0..ceiling forward then
 * reverse and latches the duty where each wheel first turns. Report-only - the
 * reporter broadcasts the four thresholds (L/R, fwd/rev). start()/stop()/active(). */
void control_deadband_start(void);
void control_deadband_stop(void);
bool control_deadband_active(void);

/* Gyro-bias calibration (any experiment): averages the gyro while held motionless
 * (~2 s) and stores the mean as the IMU's zero-rate bias; motion leaves it unchanged.
 * Requires START_CONTROL. start() arms it, the reporter broadcasts the result. */
void control_gyrocal_start(void);
bool control_gyrocal_active(void);
