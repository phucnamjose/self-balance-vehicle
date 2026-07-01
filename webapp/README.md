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

## Notes

- Serve the page over **http** (not https). A secure page cannot open a plain
  `ws://` connection to the device (mixed-content blocking).
- The ESP32 sends `Access-Control-Allow-Origin: *` on `/api` and `/ota` so this
  cross-origin page can use them; WebSocket is exempt from CORS.
- Recording: pick a topic (`imu` / `angles` / `motors`) and click **Record to
  file**. On Chromium it streams straight to disk; on Firefox/Safari it buffers
  in memory and downloads `topic_YYYYMMDD_HHMMSS.csv` when you stop.
- Telemetry batches arrive as packed binary WebSocket frames (little-endian:
  `[u8 topic_id][u8 field_count][u16 sample_count]` then per sample a `uint32`
  `t_ms` and `field_count-1` `float32`s). The `TOPIC_BY_ID` / `TOPICS` / `FMT`
  tables here must stay in sync with the per-topic packers in
  `firmware/main/telemetry.c` (`telemetry_topic_pack`).
