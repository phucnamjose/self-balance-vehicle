# XY-160D Dual DC Motor Driver

The XY-160D is the motor driver for this robot: a **dual-channel H-bridge** (L298-style
logic) that drives our **two DC gear motors** - one per wheel. The ESP32 tells it
*which direction* to spin each motor and *how fast* (via PWM).

---

## 1. Key specs (why this board)

| Parameter | Value | Why it matters here |
|-----------|-------|---------------------|
| Channels | 2 | One per wheel - exactly what we need |
| Motor supply | DC 6.5 - 27 V | Run motors from a battery (e.g. 2S/3S Li-ion, 7.4-12 V) |
| Logic supply | DC 3.3 - 5 V | Powers the board's logic side. **Power this from 3.3 V** (see note below) so the ESP32's 3.3 V signals are full-scale against the logic rail |
| Logic level | High = 3.0 - 6.5 V | At 5 V logic supply, a 3.3 V GPIO sits at the bottom of the input range and is unreliable (esp. the PWM `EN` pin). Powering the logic side at 3.3 V fixes this - no level shifter needed |
| Continuous current | 7 A / channel | Plenty for small gear motors |
| Peak current | 50 A | Handles stall/startup surges |
| PWM frequency | **0 - 10 kHz** | **Hard ceiling. Keep motor PWM <= 10 kHz** (the earlier "20 kHz" note is wrong for this board) |
| Min pulse width | ~10 us | Limits usable duty resolution at high PWM rates |

---

## 2. Pinout

### Power / output (screw terminals)

| Label | Connects to |
|-------|-------------|
| `9-24V` (VIN +) | Battery **+** (motor supply, 6.5-27 V) |
| `PGND` (VIN -) | Battery **-** |
| `OUT1` / `OUT2` | Left motor (channel 1) +/- |
| `OUT3` / `OUT4` | Right motor (channel 2) +/- |

> If a motor spins the "wrong" way, just swap its two OUT wires (or flip the
> direction in firmware). We'll lock this down during the motor test step.

### Logic / control (header pins)

| Label | Function | Drive with |
|-------|----------|-----------|
| `+5V` | Logic power in | 5 V supply |
| `GND` | Logic ground | **Must be common with ESP32 GND and PGND** |
| `IN1`, `IN2` | Channel 1 direction | ESP32 GPIO (plain digital out) |
| `ENA` (near IN1) | Channel 1 speed | ESP32 GPIO (**PWM**) |
| `IN3`, `IN4` | Channel 2 direction | ESP32 GPIO (plain digital out) |
| `ENB` (near IN3) | Channel 2 speed | ESP32 GPIO (**PWM**) |

So each motor needs **3 ESP32 pins**: 2 direction + 1 PWM = **6 pins total**.

---

## 3. How control works (truth table)

For one channel, `ENA` gates the output (PWM = speed) and `IN1`/`IN2` pick direction:

| ENA | IN1 | IN2 | Result |
|-----|-----|-----|--------|
| PWM | H | L | Spin one way, speed ∝ PWM duty |
| PWM | L | H | Spin the other way, speed ∝ PWM duty |
| x | L | L | **Brake** - motor's two terminals shorted *together* (active stop) |
| x | H | H | **Coast / isolate** - outputs high-impedance, motor free-wheels |
| 0 | x | x | **Coast** - bridge disabled, motor free-wheels |

> **Is IN1 = IN2 = HIGH safe?** Yes. It's the defined "isolate" (coast) state.
> No combination of IN/EN pins can short the power supply: each IN drives a whole
> leg through internal complementary logic, so **shoot-through is prevented in
> hardware**. The "short" in the brake row shorts the *motor terminals to each
> other* (normal dynamic braking), not the supply.

**The control scheme we'll use in firmware:**
- `ENA`/`ENB` = LEDC PWM, duty = |commanded effort| in 0..100 %.
- Direction pins:
  - forward -> `(IN1,IN2) = (1,0)`
  - reverse -> `(IN1,IN2) = (0,1)`
  - stop    -> `(IN1,IN2) = (0,0)` (brake)

A signed command from the controller (e.g. PID output in -1..+1) maps cleanly:
the **sign** sets the direction pins, the **magnitude** sets the PWM duty.

---

## 4. Wiring to the ESP32 (proposed)

These GPIOs avoid the strapping pins (0, 2, 5, 12, 15), the flash pins (6-11),
and the I2C bus (21/22). Adjust to match your wiring; they are `#define`s in
firmware.

| Signal | ESP32 GPIO | Driver pin |
|--------|-----------|------------|
| Left PWM | GPIO25 | ENA |
| Left dir A | GPIO26 | IN1 |
| Left dir B | GPIO27 | IN2 |
| Right PWM | GPIO33 | ENB |
| Right dir A | GPIO32 | IN3 |
| Right dir B | GPIO14 | IN4 |
| Logic ground | GND | GND |

### Wheel encoders (quadrature A/B -> PCNT)

The encoders use **pull-up-capable** GPIOs (not the input-only 34-39, which have
no internal pull-ups - many encoder outputs are open-collector and need one).
Each A/B pair drives one PCNT unit configured for 4x decoding.

| Signal | ESP32 GPIO |
|--------|-----------|
| Left encoder A | GPIO18 |
| Left encoder B | GPIO19 |
| Right encoder A | GPIO23 |
| Right encoder B | GPIO13 |

```
                 +-------------------+
  Battery + ---->| 9-24V             |
  Battery - --+->| PGND         OUT1 |---- Left motor +
              |  |              OUT2 |---- Left motor -
              |  |              OUT3 |---- Right motor +
              |  |              OUT4 |---- Right motor -
              |  |                   |
  ESP32 3V3 ---->| +5V             |   (logic supply: use 3.3 V, see notes)
              +->| GND               |   (common ground!)
                 |  IN1 IN2 ENA      |
                 |  IN3 IN4 ENB      |
                 +---|---|---|-------+
   ESP32             26  27  25   (left:  IN1 IN2 ENA)
   ESP32             32  14  33   (right: IN3 IN4 ENB)
   ESP32 GND --------------------------- common ground
```

---

## 5. Power & safety notes

- **The inputs are optocoupler-isolated - the logic-supply pin MUST be powered.**
  Unlike an L298N (which makes its own 5V from the motor supply), this board's
  input/opto side is dead until you feed its logic `+5V`/`GND` header. **No logic
  supply = motors never move, even with perfect signals.** This is the #1 "it
  doesn't work".
- **Power the logic side from 3.3 V, not 5 V (verified on this build).** With the
  logic rail at 5 V, the ESP32's 3.3 V `IN`/`EN` outputs sit at the bottom of the
  input range and don't reliably trigger the opto inputs - the symptom is the
  `motor ... = N%` reply printing while the wheels stay dead. Feeding the driver's
  logic `+5V` pin from the ESP32 `3V3` instead makes the 3.3 V signals full-scale
  against the logic rail and they trigger reliably, with no level shifter. (If you
  must keep the logic rail at 5 V, drive IN/EN through a 3.3->5 V **non-inverting**
  buffer such as a 74HCT244/245 - HCT, not HC, for the 2.0 V input threshold.)
- **Common ground is mandatory.** ESP32 GND, driver `GND`/`+5V` return, and the
  battery `PGND` must all be tied together, or the logic signals have no reference
  and the motors won't respond (a very common "it doesn't work" cause).
- **Motor supply must be >= 6.5 V.** The board has undervoltage protection; a 3.7 V
  or 5 V bench supply on `9-24V` will not spin the motors. Use a real battery/PSU.
- **Separate the motor supply from the ESP32 supply.** Motors create voltage sags
  and noise spikes that can brown-out or reset the ESP32. Power the ESP32 from USB
  or its own regulator, and the motors from the battery; just share ground.
- **Fuse it.** The board can pass 50 A peak; a ~10 A fuse in series with the motor
  supply protects against a shorted/stalled motor. Never short the OUT terminals.
- **Don't exceed 27 V and don't reverse polarity** on the motor supply - either can
  destroy the board.
- **PWM frequency <= 10 kHz.** We'll likely use ~1-10 kHz. Lower is gentler on the
  H-bridge but can whine audibly; we'll pick a value during the motor test.
- **Expect a deadband.** Below some minimum duty the motor won't turn (friction +
  H-bridge drop). We'll measure this during testing and compensate in firmware.

---

## 6. Where this fits in the roadmap (Phase 4)

1. **Make a program to test PWM with a real motor** (done) - `ENA`/`IN` pins spin
   each motor open-loop from a `motor <l|r|both> <-100..100>` command.
2. **Test the real motor** - find direction, deadband, and max usable speed.
3. **Read encoder** (done) - quadrature A/B into the PCNT hardware counter; total
   counts + counts/sec stream in telemetry. `enc reset` zeroes them.
4. **PC commands** - `motor`/`stop` set PWM; `enc reset` clears the count.

> Firmware note: motor PWM runs on its own LEDC timer (`LEDC_TIMER_1`) and channels
> for `ENA`/`ENB` at 10 kHz, separate from the LED pin.
