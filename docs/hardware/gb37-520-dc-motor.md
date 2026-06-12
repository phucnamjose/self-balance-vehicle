# GB37-520 DC Gear Motor (with Hall Encoder)

The two wheels are driven by **GB37-520 DC servo gear motors**: a 12 V brushed
motor, a 30:1 gearbox, and a built-in **2-channel Hall quadrature encoder**. The
encoder is what lets us measure each wheel's **speed, direction, and position** -
the feedback the balance controller will eventually need.

One per wheel, both driven by the [XY-160D](xy-160d-motor-driver.md) and read by
the ESP32's PCNT counter.

---

## 1. Motor specs

| Parameter | Value | Notes |
|-----------|-------|-------|
| Model | GB37-520 DC servo geared motor | 37 mm gear motor family |
| Operating voltage | **12 V DC** | Set the motor supply to ~12 V |
| Gear ratio | **30:1** | Output shaft turns 30x slower than the motor |
| No-load speed | 333 RPM | Output shaft, at 12 V |
| Max speed under load | 250 RPM | Output shaft |
| No-load current | 120 mA | |
| Max (stall) current | 1 A | Well within the XY-160D's 7 A/channel |
| Rated torque | 3.5 kg·cm | |
| Max torque | 5 kg·cm | |
| Motor diameter | 37 mm | |
| Shaft diameter | 6 mm | |
| Gearbox length | 22 mm | |

> The 1 A max current means a single ~2-3 A fuse per motor (or one 10 A on the
> shared supply) is plenty. The XY-160D is hugely over-spec'd for these, which is
> fine - it just runs cool.

---

## 2. Built-in encoder

| Parameter | Value |
|-----------|-------|
| Type | Hall sensor, **2 channels (A / B)** - quadrature |
| Encoder supply | **3.3 - 5 V DC** |
| Pulses per motor-shaft revolution | **11 per channel** (22 counting both A+B) |
| Pulses per **output-shaft** revolution | 11 x 30 = **330 per channel** |

The encoder sits on the **motor shaft** (before the gearbox), so the gearbox
multiplies its resolution by 30 at the wheel.

### What the ESP32 actually counts (4x decoding)

Our firmware uses the **PCNT peripheral in 4x quadrature mode** - it counts *every*
edge of *both* channels. So multiply the single-channel pulse count by 4:

| Decoding | Counts / motor rev | Counts / **output (wheel) rev** |
|----------|--------------------|---------------------------------|
| 1x (one edge, one channel) | 11 | 330 |
| 2x | 22 | 660 |
| **4x (what we use)** | **44** | **1320** |

So **1320 encoder counts = one full wheel revolution.** This is the number to use
when converting counts to distance or angle.

### Counts <-> speed / position

The firmware reports `posL`/`posR` (total counts) and `velL`/`velR` (counts per
second, measured over a 1 s window). To convert:

```
wheel revolutions      = counts / 1320
wheel RPM              = (counts_per_sec * 60) / 1320  =  counts_per_sec / 22
wheel angle (deg)      = (counts / 1320) * 360         =  counts * 0.2727
```

Sanity check at top speed: 333 RPM no-load -> ~7300 counts/s; 250 RPM loaded ->
~5500 counts/s. The PCNT counter rolls over (and our ISR accumulates) every
~4-5 s at full tilt, which is well within the hardware's capability.

---

## 3. Pinout (6-wire motor)

The motor has **6 wires**: two for motor power, four for the encoder.

| Wire color | Label | Function | Connects to |
|------------|-------|----------|-------------|
| **Red** | M1 | Motor power | XY-160D `OUT` (one terminal of the channel) |
| **White** | M2 | Motor power | XY-160D `OUT` (other terminal of the channel) |
| **Black** | GND | Encoder ground (0 V) | ESP32 `GND` |
| **Blue** | VCC | Encoder supply (3.3-5 V) | ESP32 **`3V3`** (see note) |
| **Yellow** | C1 / A | Encoder channel A | ESP32 encoder A pin |
| **Green** | C2 / B | Encoder channel B | ESP32 encoder B pin |

> **Power the encoder from 3.3 V, not 5 V.** The ESP32 GPIOs are **not 5 V
> tolerant**. If the Hall outputs swing to 5 V they can damage the pin. Feeding
> `VCC` (blue) from the ESP32's `3V3` makes the A/B signals 3.3 V - safe to read
> directly. (Our firmware also enables internal pull-ups on the A/B pins.)

> **M1/M2 are not polarity-critical** - swapping them just reverses the spin
> direction. We'll align "positive command = forward" and "encoder counts up =
> forward" in software during testing; if a wheel's encoder counts *down* when it
> drives forward, swap that motor's A/B (yellow/green) wires or its M1/M2.

### Mapping to ESP32 (matches the firmware)

| Motor | Power (M1/M2) | Encoder A (yellow) | Encoder B (green) |
|-------|---------------|--------------------|-------------------|
| Left  | XY-160D OUT1/OUT2 | GPIO18 | GPIO19 |
| Right | XY-160D OUT3/OUT4 | GPIO23 | GPIO13 |

Encoder `GND` (black) -> ESP32 `GND`; encoder `VCC` (blue) -> ESP32 `3V3`.

```
   GB37-520 (x2)                 XY-160D                  ESP32
   ------------                  -------                  -----
   Red  (M1) ───────────────────  OUTx
   White(M2) ───────────────────  OUTx
   Blue (VCC) ───────────────────────────────────────────  3V3
   Black(GND) ───────────────────────────────────────────  GND
   Yellow(A) ────────────────────────────────────────────  GPIO18 / 23
   Green (B) ────────────────────────────────────────────  GPIO19 / 13
```

---

