# ESP32 Development Board

The ESP32 is the brain of this robot: it runs the real-time balance loop, reads
the IMU and encoders, drives the motors, and hosts its own Wi-Fi access point
for logging and OTA updates. This page introduces the board and records **how we
allocate its pins** so we never double-book one.

We target a generic **ESP32 DevKit** (ESP32-WROOM-32, 4 MB flash) - the common
38-pin dev board. A WROVER works too, but see the GPIO16/17 note below.

---

## 1. Key specs (why this chip)

| Parameter | Value | Why it matters here |
|-----------|-------|---------------------|
| CPU | Dual-core Xtensa LX6 @ 240 MHz | One core runs the control loop, the other runs Wi-Fi/HTTP - no contention |
| RAM | 520 KB SRAM | Comfortable for FreeRTOS + Wi-Fi stack + our buffers |
| Flash | 4 MB (typical) | Fits two 1.5 MB OTA app slots (see `partitions.csv`) |
| Wi-Fi | 802.11 b/g/n | Hosts a SoftAP for the dashboard + OTA - no USB cable needed |
| Bluetooth | BT 4.2 / BLE | Unused for now |
| Logic level | **3.3 V** | All GPIOs are 3.3 V; **not 5 V tolerant** - feed sensors/encoders 3.3 V logic |
| GPIO current | ~12 mA typical / 40 mA abs max per pin | Drive logic, not motors - motors go through the XY-160D |
| ADC / DAC | 18 ch ADC, 2 ch DAC | Available for battery-voltage sensing later |
| Timers | 4 hardware (GPTimer) + PCNT + LEDC | We use GPTimer (loop tick), LEDC (motor PWM), PCNT (encoders) |

---

## 2. Power

| Pin | Use |
|-----|-----|
| `5V` / `VIN` | 5 V **in** (from USB or a regulated 5 V rail). The onboard regulator makes 3.3 V |
| `3V3` | 3.3 V **out** - powers the MPU6050 and encoder pull-ups (keep total draw modest) |
| `GND` | Ground - **must be common** with the motor driver's logic ground |

> The ESP32 powers logic only. Motors are powered from the battery **through the
> XY-160D**, never from the board's 3V3/5V pins. Share a common ground.

Wi-Fi transmit can spike current draw; power the board from a source that can
supply ~500 mA so brownouts don't reset it mid-run.

---

## 3. GPIO map for this project

Our committed pin allocation. Anything not listed is free for later use.

| GPIO | Used for | Notes |
|------|----------|-------|
| 0 | BOOT button | Strapping pin; also the boot-mode select |
| 13 | Right encoder B | Pull-up capable |
| 14 | Motor R IN4 (dir) | Direction output |
| 16 | Right LED / notify | Plain digital out |
| 17 | Left LED / notify | Plain digital out |
| 18 | Left encoder A | Pull-up capable |
| 19 | Left encoder B | Pull-up capable |
| 21 | I2C SDA | To MPU6050 |
| 22 | I2C SCL | To MPU6050 |
| 23 | Right encoder A | Pull-up capable |
| 25 | Motor L ENA (PWM) | LEDC channel |
| 26 | Motor L IN1 (dir) | Direction output |
| 27 | Motor L IN2 (dir) | Direction output |
| 32 | Motor R IN3 (dir) | Direction output |
| 33 | Motor R ENB (PWM) | LEDC channel |
| 1, 3 | UART0 TX/RX | USB serial console + flashing - leave alone |
| 6-11 | SPI flash | **Reserved - do not use** |

### Pin gotchas

- **Strapping pins** (0, 2, 5, 12, 15): their level at boot selects boot mode and
  flash voltage. Avoid driving 12 and 15 externally at reset; 0 is fine as the
  BOOT button. GPIO2 is now free (no longer used for the onboard LED).
- **Input-only pins** (34, 35, 36, 39): no output drivers **and no internal
  pull-ups/pull-downs**. We avoided them for encoders because open-collector
  encoder outputs need a pull-up - regular GPIOs (13/18/19/23) give us that for
  free.
- **GPIO16 / 17**: free on WROOM, but used for PSRAM on WROVER modules. We drive
  the Right (16) and Left (17) notification LEDs here, so use a WROOM module.
- All pins are **3.3 V**. Anything feeding a GPIO (sensor, encoder) must use 3.3 V
  logic or be level-shifted.

---

## 4. Full board pinout (compare against our map)

Physical header of the **ESP32-WROOM-32 DevKit (38-pin)**, oriented with the
**antenna/shield at the top and the USB connector at the bottom**. Pin numbers run
down the left side (1-19) and back up the right side (20-38). Markers: `*strap` =
strapping pin, `(in)` = input-only, `flash` = wired to the SPI flash (do not use).
Our project's use is shown next to each pin.

```
   left  (GPIO  use)        pin        pin  (GPIO  use)  right
   ----------------------   ---        ---  ----------------------
                  3V3        1          38   GND
                   EN        2          37   GPIO23  R enc A
        GPIO36 VP (in)       3          36   GPIO22  I2C SCL
        GPIO39 VN (in)       4          35   GPIO1   TX0 (serial)
        GPIO34    (in)       5          34   GPIO3   RX0 (serial)
        GPIO35    (in)       6          33   GPIO21  I2C SDA
        GPIO32  R IN3        7          32   GND
        GPIO33  R PWM        8          31   GPIO19  L enc B
        GPIO25  L PWM        9          30   GPIO18  L enc A
        GPIO26  L IN1       10          29   GPIO5   *strap
        GPIO27  L IN2       11          28   GPIO17  L LED
        GPIO14  R IN4       12          27   GPIO16  R LED
        GPIO12  *strap      13          26   GPIO4
        GND                 14          25   GPIO0   *strap (BOOT)
        GPIO13  R enc B     15          24   GPIO2   *strap
        GPIO9   flash       16          23   GPIO15  *strap
        GPIO10  flash       17          22   GPIO8   flash
        GPIO11  flash       18          21   GPIO7   flash
        VIN (5V)            19          20   GPIO6   flash
                            └────[ USB ]────┘
```

## 5. Flashing & console

- **USB**: a CP2102/CH340 USB-UART bridge on the board exposes UART0 (GPIO1/3) as
  a COM port. `idf.py -p COMx flash monitor` builds, flashes, and opens the serial
  log at 115200 baud.
- **OTA**: once running, the board hosts a Wi-Fi AP and serves a web page with a
  firmware-upload control. New images land in the spare app slot, so a bad upload
  can't brick the running firmware. See `partitions.csv` and the OTA notes in
  `app_main.c`.
- **BOOT + EN**: if auto-flash ever fails, hold **BOOT**, tap **EN** (reset),
  release BOOT to force the ROM bootloader.

---
