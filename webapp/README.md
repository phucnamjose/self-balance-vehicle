# Balance Bot web app

The control UI used to be baked into the firmware and served by the ESP32. It
now lives here and runs on your PC; the ESP32 only exposes the telemetry API
(`/ws`, `/api/telemetry`, `/ota`). You enter the device IP in the page and it
connects over WebSocket.

## Run it

Serve this folder over plain HTTP from your PC and open it in a browser.

```bash
cd webapp
python3 -m http.server 8000
```

Then open <http://localhost:8000/>.

Any static file server works (`npx serve`, VS Code Live Server, etc.). Use
`localhost`/`127.0.0.1` rather than your machine's Wi-Fi IP - loopback traffic
never leaves the PC, so loading the page does not round-trip through the ESP32.

## Connect to the robot

1. Join the robot's Wi-Fi access point (it is its own AP).
2. In the page, enter the ESP32 IP (default `192.168.4.1`) and click **Connect**.
   The IP is remembered between sessions.

Only the data connection (`ws://<ip>/ws`) and OTA upload (`http://<ip>/ota`)
travel over Wi-Fi to the device; everything else is local.

## Panels

The top card (connection, control-task START/STOP, live IMU/motor/encoder
readouts) is always visible; the terminal + data log sit at the bottom. In
between are the tabs, each matching an experiment mode:

1. **Manual** (`TEST_MOTORS`) - open-loop duty bar (`l`/`r`/`both` + slider),
   the deadband sweep (**Find deadband**), and the playback script uploader.
2. **Motors controller** (`TEST_MOTOR_CONTROLLERS`) - per-wheel speed setpoint
   bar (`speed <l|r|both> <rad/s>`), live PI-gain editor (`gains ...`), a
   setpoint slew-rate limit (`slew <rad/s^2>`: ramps the setpoint to kill the
   step "kick" - the recommended overshoot fix), a setpoint-weight `b`
   slider (`bweight <0..1>`, keep at 1 for the full speed range), a
   speed-measurement filter box (`tauf <s>`: higher smooths the noisy encoder
   speed but adds lag/overshoot), numeric setpoint/measured readouts, and two
   live plots (last 10 s): measured vs.
   setpoint wheel speed, and the motor output (applied duty vs. the raw PI
   command before deadband compensation and saturation).
3. **Angle Estimator** (`TEST_ANGLES_ESTIMATION`) - the firmware tilt estimate:
   pitch readout, complementary-filter `alpha` editor, gyro calibration, an **IMU
   pitch offset** box (`imuoffset <deg>`: the mounting tilt vs. true horizontal,
   subtracted from the estimate so a level robot reads 0; default -3 deg), the
   attitude vector view, and a pitch plot.
4. **Balance** (`TEST_BALANCE`) - the self-balancing tilt loop
   (`docs/theory/balance-controller.md`). **Start/STOP balancing** buttons
   (`balance on`/`off`), the balance-PID tuning boxes (`bgains <kp> <ki> <kd>`,
   trim `btrim`), and four live views reused from the other tabs: the attitude
   vector, the pitch plot, the wheel-speed plot (setpoint = the balance command
   `w_common`), and the motor-output plot. Entering the tab is **safe** - it
   turns estimation on but leaves the balance loop off until you click **Start
   balancing** (the motors do not move on tab switch). Gains are untuned seeds.
5. **Flashing** - the OTA `.bin` upload (plus STOP_CONTROL and rollback).

Selecting the **Manual**, **Motors controller** or **Angle Estimator** tab also
sends the matching `exp` command so the panel's controls take effect. You still
need the control task running (**START**, top bar); flashing needs it stopped
(**STOP**).

## Notes

- Terminal: type commands (try `help`) and press Enter or **Send**; replies and
  warnings are timestamped in the data log. **Clear log** empties the log view
  (device state is untouched).
- Serve the page over **http** (not https). A secure page cannot open a plain
  `ws://` connection to the device (mixed-content blocking).
- The ESP32 sends `Access-Control-Allow-Origin: *` on `/api`, `/ota`,
  `/motorseq` and `/speedseq` so this cross-origin page can use them; WebSocket
  is exempt from CORS.
- Recording: pick a topic (`imu` / `angles` / `motors`) and click **Record to
  file**. On Chromium it streams straight to disk; on Firefox/Safari it buffers
  in memory and downloads `topic_YYYYMMDD_HHMMSS.csv` when you stop. CSV values
  are written to 6 decimal places (the `t` column is seconds).
- Motor script: in `TEST_MOTORS` (send `exp motors`) with the control task
  running, pick a `.csv`/`.txt` file and click **Load & play**. The file is one
  step per line, `dur,mL,mR`: `dur` is how long to hold that output in seconds
  (> 0) and `mL,mR` are each `-1..1` (`#` comments and blank lines ignored; up to
  1 minute total). The player runs each step for its duration in turn, then parks
  the motors. **Stop** aborts it. Example:

  ```
  # dur(s), mL, mR
  2.0, 0.3, 0.3
  2.0, 0.5, 0.5
  1.0, -0.5, -0.5
  ```
- Speed script: the same uploader in the **Motors controller** panel
  (`TEST_MOTOR_CONTROLLERS`, send `exp motor-ctrl`) posts to `/speedseq`
  instead. Columns are `dur,wL,wR` with `wL,wR` as wheel-speed setpoints in
  `rad/s`; each step is driven through the wheel PI, so a script of speed steps
  is a reproducible step input for tuning (e.g. measuring overshoot). The player
  zeroes the setpoints at the end. Example:

  ```
  # dur(s), wL(rad/s), wR(rad/s)
  1.0, 0, 0
  2.0, 10, 10
  2.0, 20, 20
  2.0, 0, 0
  ```
- Telemetry batches arrive as packed binary WebSocket frames (little-endian:
  `[u8 topic_id][u8 field_count][u16 sample_count]` then per sample a `uint64`
  `t_us` and `field_count-1` `float32`s). The `TOPIC_BY_ID` / `TOPICS` tables
  here must stay in sync with the per-topic packers in
  `firmware/main/telemetry.c` (`telemetry_topic_pack`).
