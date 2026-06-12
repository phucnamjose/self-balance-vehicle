# Project Roadmap & Progress Tracker

Single source of truth for what we are building and where we are. We work
**one step at a time**: pick the next unchecked item, build a small focused
example, verify it on hardware, check it off, and write a short note.

Status legend: `[ ]` todo &nbsp; `[~]` in progress &nbsp; `[x]` done

> Current focus: **Phase 4 - Step 2: test the real motor** (direction, deadband, max speed).
> Open-loop driver is in: `motor <l|r|both> <-100..100>` / `stop` from the web terminal.
> Code: `firmware/main/app_main.c`, `www/index.html`. Driver doc: `docs/hardware/xy-160d-motor-driver.md`.

---

## Phase 1 - Learn how to code ESP32
Goal: get comfortable with the ESP-IDF basics, one peripheral at a time.

- [x] Install the IDE / ESP-IDF toolchain
- [x] Run "Hello world"
- [x] Use GPIO (blink an LED, read a button)
- [x] Use the Timer (periodic callback) - hardware GPTimer + control task
- [x] Generate PWM (LEDC) - duty driven from the control tick
- [x] Read I2C (scan the bus) - boot-time probe finds the MPU6050
- [x] Use multi-tasking (FreeRTOS tasks, queues) - control on core 1, reporter on core 0

## Phase 2 - Build wireless log and upload code
Goal: develop without a USB cable - see logs and flash over Wi-Fi.

- [x] Wireless log in a webpage
- [x] Wireless log, interactive from a PC terminal (web-page terminal over WebSocket)
- [x] Wireless uploading code (OTA)

## Phase 3 - Read MPU6050
Goal: trustworthy IMU data. (Calibration moved to Phase 8.)

- [x] Read MPU6050 (raw accel/gyro over I2C)

## Phase 4 - Generate PWM to control motors
Goal: drive the real motors and command them remotely.

- [x] Test PWM with 2 real motors
- [~] Add a PC command to test the motor (direction, deadband, max speed)
- [ ] Read encoder using timer
- [ ] Add a PC command to reset encoder count

## Phase 5 - Build basic web app: states + log
Goal: a dashboard to observe and poke the robot.

- [ ] Show the states
- [ ] Show the log in a log tab
- [ ] Add a testing tab to test the PWM
- [ ] Draw a line chart

## Phase 6 - Put ESP32 + MPU6050 on the robot
Goal: physical integration.

- [ ] Fix the kit and sensor on the robot
- [ ] Connect the bus and lines (wiring)

## Phase 7 - Investigate the maths & algorithms
Goal: understand the model before controlling it.

- [ ] Inverted pendulum model
- [ ] Kalman filter for state estimation

## Phase 8 - Simulate the model in MATLAB
Goal: validate the model and tune control offline.

- [ ] Build and run the MATLAB simulation

## Phase 9 - Run real system
Goal: make the robot stand stably

- [ ] Calibrate the sensor (offsets / bias)
- [ ] Apply estimation
- [ ] Apply control

---

## Progress log
Newest entries on top. Keep each note to a line or two: what we did, what we
learned, where the code lives.

| Date | Phase / Step | Notes |
|------|--------------|-------|
| 2026-06-12 | P2 (fix) | Fixed WS lag/`error in send: 11`: the blocking `httpd_ws_send_frame_async` ran inside reporter_task, so a slow client stalled it ~5 s (logs+web only updated every ~6 s). Now reporter strdups the frame and `httpd_queue_work`s it to the server task (non-blocking); failed sends drop the client (`httpd_sess_trigger_close`). Also `send_wait_timeout=2`, `lru_purge_enable=true`. Control loop was never affected. |
| 2026-06-12 | P4 S1 | Dropped the LED PWM fade demo; GPIO2 is now a plain output (led_init/led_set) reserved for notification blinks. Removed the obsolete "duty" telemetry field everywhere (motors' mL/mR are the real actuator readout now). |
| 2026-06-12 | P4 S1 | Open-loop motor driver: motor_init() sets up 2 LEDC PWM channels (ENA/ENB on GPIO25/33, timer 1, 10 kHz) + 4 direction GPIOs (26/27/32/14). motor_set(i,cmd) maps sign->dir, magnitude->duty. control_task applies s_motor_cmd[] each tick. Web terminal: "motor <l|r|both> <-100..100>" / "stop", plus a slider+STOP panel and mot=[L R]% in telemetry. Motors start stopped. |
| 2026-06-12 | P4 | Documented the XY-160D dual H-bridge in docs/hardware/: specs, pinout, truth table, control scheme (PWM on ENA/ENB + direction on IN pins), proposed ESP32 wiring, safety. Key facts: 3.3V-logic compatible (direct GPIO drive), 7A/ch, PWM ceiling 10 kHz (NOT 20). Jumped here from P3; the P3 query-sensor command is deferred. |
| 2026-06-12 | P3 | Reverted the calibration code (gyro bias + accel zero) and moved that task to Phase 8 - it'll be more meaningful once we have the model/sim and the IMU is mounted at its real (non-flat) orientation. Kept the unrelated 4 ms I2C-timeout safety fix. |
| 2026-06-11 | P3 (safety) | Bounded the IMU I2C timeout to 4 ms (was 100 ms) so a bus glitch costs ~1 tick, not a 100 ms loop stall. Discussed read-in-loop vs split estimator rate: keeping 200 Hz combined; event-driven INT/FIFO deferred to the Kalman phase. |
| 2026-06-11 | P3 S1 | Read MPU6050: added it as an I2C device, wake via PWR_MGMT_1, configure +/-2g/+/-250dps + DLPF. control_task reads the 14-byte block each tick, converts to g/dps/degC, ships it in telemetry. Web page + serial log now show accel/gyro/temp. Watch dt_min/max - the ~0.4ms I2C read sits inside the 5ms budget. Also added "rollback" WS command (switch to other OTA slot). |
| 2026-06-11 | P2 S3 | OTA over Wi-Fi: added partitions.csv (4 MB, two 1.5 MB app slots ota_0/ota_1) + /ota POST handler that streams the .bin into the spare slot, validates, flips boot, reboots. Web page got a "Flash" upload control. The old 1 MB/20%-free limit is gone - each slot now holds ~1.5 MB. Running slot logged at boot. |
| 2026-06-11 | P2 S2 | Added WebSocket /ws: server streams telemetry + accepts text commands (help/stats/stream). Web page gained a terminal. Chose WS over UDP because browsers can't do raw UDP. |
| 2026-06-11 | P2 S1 | Web page deduped (log line per change); moved HTML to www/index.html embedded via EMBED_TXTFILES. Note: log history lives in the browser, ESP keeps only the latest snapshot. |
| 2026-06-11 | P2 S1 | Phase 1 done. Started Wi-Fi: SoftAP "balance-bot" + HTTP server on core 0; page polls /api/telemetry and shows the live log. reporter_task now caches the latest snapshot for the server. |
| 2026-06-11 | P1 S7 | I2C scan verified. Multi-task split: control_task (core 1) pushes a telemetry snapshot to a 1-slot queue each second; reporter_task (core 0) consumes + logs. Control loop now does zero slow work - ready for Wi-Fi on core 0. |
| 2026-06-11 | P1 S6 | PWM verified (LED fades). Added I2C master bus + boot-time scan (probe addr 1..127); expect MPU6050 at 0x68. Bus handle kept for the next step (reading the IMU). |
| 2026-06-10 | P1 S5 | Timer verified (tight jitter). Added LEDC PWM on the LED pin; control_task computes a triangle-wave duty each tick to fade the LED - foundation for motor speed control. |
| 2026-06-08 | P1 S4 | Switched to a hardware GPTimer for deterministic timing: alarm ISR notifies a high-priority control task at a fixed rate (the real control-loop pattern). Measures loop jitter. |
| 2026-06-08 | P1 S4 | GPIO verified. Started Timer: a periodic `esp_timer` first drove the LED blink; BOOT restarts it at a different period. |
| 2026-06-08 | P1 S3 | Hello world verified on hardware. Started GPIO: blink LED on GPIO2, read BOOT button on GPIO0 to toggle blink rate. |
| 2026-06-07 | P1 S1 | Updated install guide to ESP-IDF v6.x (v6.0.1) via the new Installation Manager (EIM). |
| 2026-06-07 | P1 S1-S2 | Wrote Windows install guide; reset `firmware/` to a minimal Hello world (heartbeat log). Ready to install + flash. |
| 2026-06-07 | Setup | Created this roadmap; deleted the big scaffold; restarting step-by-step as one evolving project. |
