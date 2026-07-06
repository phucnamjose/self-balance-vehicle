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
between are three tabs, each matching an experiment mode:

1. **Manual** (`TEST_MOTORS`) - open-loop duty bar (`l`/`r`/`both` + slider),
   the deadband sweep (**Find deadband**), and the playback script uploader.
2. **Motors controller** (`TEST_MOTOR_CONTROLLERS`) - per-wheel speed setpoint
   bar (`speed <l|r|both> <rad/s>`), live PI-gain editor (`gains ...`), numeric
   setpoint/measured readouts, and two live plots (last 10 s): measured vs.
   setpoint wheel speed, and the motor output (applied duty vs. the raw PI
   command before deadband compensation and saturation).
3. **Flashing** - the OTA `.bin` upload (plus STOP_CONTROL and rollback).

Selecting the **Manual** or **Motors controller** tab also sends the matching
`exp` command so the panel's controls take effect. You still need the control
task running (**START**, top bar); flashing needs it stopped (**STOP**).

## Notes

- Serve the page over **http** (not https). A secure page cannot open a plain
  `ws://` connection to the device (mixed-content blocking).
- The ESP32 sends `Access-Control-Allow-Origin: *` on `/api`, `/ota` and
  `/motorseq` so this cross-origin page can use them; WebSocket is exempt from CORS.
- Recording: pick a topic (`imu` / `angles` / `motors`) and click **Record to
  file**. On Chromium it streams straight to disk; on Firefox/Safari it buffers
  in memory and downloads `topic_YYYYMMDD_HHMMSS.csv` when you stop. CSV values
  are written to 6 decimal places (the `t` column is seconds).
- Motor script: in `TEST_MOTORS` (send `exp motors`) with the control task
  running, pick a `.csv`/`.txt` file and click **Load & play**. The file is one
  step per line, `dur,mL,mR`: `dur` is how long to hold that output in seconds
  (> 0) and `mL,mR` are each `-1..1` (`#` comments and blank lines ignored; at
  most 20 steps). The player runs each step for its duration in turn, then parks
  the motors. **Stop** aborts it. Example:

  ```
  # dur(s), mL, mR
  2.0, 0.3, 0.3
  2.0, 0.5, 0.5
  1.0, -0.5, -0.5
  ```
- Telemetry batches arrive as packed binary WebSocket frames (little-endian:
  `[u8 topic_id][u8 field_count][u16 sample_count]` then per sample a `uint64`
  `t_us` and `field_count-1` `float32`s). The `TOPIC_BY_ID` / `TOPICS` tables
  here must stay in sync with the per-topic packers in
  `firmware/main/telemetry.c` (`telemetry_topic_pack`).
