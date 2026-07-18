/**
 * @file control.h
 * @brief Hard real-time control loops (core 1) and telemetry reporter (core 0).
 *
 * Two core-1 tasks, each woken by its own GPTimer ISR: motor_task at CONTROL_HZ
 * (encoders, wheel PI, motor output, telemetry - never touches I2C) and the lower
 * priority imu_task at IMU_HZ (MPU6050 read, estimator, balance PID). imu_task
 * publishes the common wheel-speed setpoint the motor loop tracks. motor_task hands a
 * batch to reporter_task (core 0), which caches, logs and streams it to WebSocket clients.
 */
#pragma once

#include <stdbool.h>

/* Control lifecycle modes (apply to both core-1 tasks together). */
typedef enum {
    STOP_CONTROL,    /* both tasks fully stopped (deleted, not suspended), motors off */
    START_CONTROL,   /* both tasks running */
} control_mode_t;

/* Create the telemetry queue + reporter task and start the control tasks
 * (START_CONTROL). Call once at boot after the web server is up. */
void control_start(void);

/* Current lifecycle mode. */
control_mode_t control_mode(void);

/* Rolling loop-timing averages from the last ~1 s window: mean motor- and IMU-task run
 * time [us] and mean new IMU samples per IMU tick (~2 at 500 Hz sampling / 250 Hz read).
 * Any pointer may be NULL. Also printed periodically to the log + web terminal. */
void control_loop_stats(float *motor_run_us, float *imu_run_us, float *imu_samples);

/* Switch lifecycle mode. STOP_CONTROL deletes both tasks at a safe point and forces
 * the motors off (required before an OTA flash); START_CONTROL (re)starts them.
 * Peripherals and the GPTimers are initialised once and reused across restarts. */
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

/* Scripted playback: a list of timed steps "hold (vL,vR) for dur seconds", replayed
 * by the motor loop. The kind selects how each step's values are applied:
 *   PLAY_DUTY  - open-loop duty (-1..1), TEST_MOTORS (controller off)
 *   PLAY_SPEED - wheel-speed setpoints (rad/s) fed to the PI, TEST_MOTOR_CONTROLLERS
 * Fill incrementally: begin(kind) clears + sets the kind, append() adds a step
 * (returns the new count; -1 step array full, -2 if dur <= 0, -3 if it would exceed the
 * 1-minute total cap), start() plays, stop() halts and zeroes the output.
 * active()/len()/pos() report progress. */
typedef enum { PLAY_DUTY, PLAY_SPEED } playback_kind_t;
void control_playback_begin(playback_kind_t kind);
int  control_playback_append(float dur, float vL, float vR);
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
