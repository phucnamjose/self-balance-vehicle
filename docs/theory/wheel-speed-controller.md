# Wheel Speed Controller

The inner loop of the cascade (see [README.md](README.md)). One instance per
wheel regulates that wheel's **angular speed** to a commanded setpoint, so the
two non-identical motors behave identically to whatever sits above them.

> Status: **design** (this step). Firmware is not written yet; the existing
> control loop is still open-loop
> ([../../firmware/main/control.c](../../firmware/main/control.c) lines 80-85).

## Purpose

Regulate one wheel's angular speed to a setpoint, independently of the other
wheel. This rejects the per-motor differences (unit variation, gearbox friction,
deadband, back-EMF) so an upper loop can command a single common speed and trust
that **both wheels actually reach it** - the robot drives straight instead of
veering.

## Interface

| | Quantity | Unit | Source / sink |
|---|----------|------|---------------|
| Input | speed setpoint `w_set` | rad/s | upper loop (manual web command for bring-up) |
| Input | measured speed `w_meas` | rad/s | encoders (per-tick rate) |
| Input | `dt` | s | control tick, `1/CONTROL_HZ = 0.005 s` |
| Output | `duty` | `[-1, 1]` | `motor_set(i, duty)` ([../../firmware/main/motors.h](../../firmware/main/motors.h)) |

- One instance per wheel, index `0 = L`, `1 = R` (same indexing as the rest of
  the firmware).
- Runs every control tick at `CONTROL_HZ` (200 Hz,
  [../../firmware/main/telemetry.h](../../firmware/main/telemetry.h)).
- `w_meas` comes from `encoder_cps_to_radps()`
  ([../../firmware/main/encoders.c](../../firmware/main/encoders.c)).

## Control law

Continuous-domain (s-domain) block diagram of one wheel's loop. `C(s)` is the PI
controller, `F(s)` the feedforward, `N(.)` the (memoryless) deadband-compensation
+ output saturation nonlinearity, and `G_m(s)` the motor+wheel transfer function
from duty to angular speed. Encoder feedback is idealized as unity:

```text
                       +---------------------+
              w_set --->|  F(s) = 1/w_noload  |--------- u_ff --------+
                |        |    (feedforward)    |                      |
                |        +---------------------+                      |
                |                                                     v
                |   e     +-------------------+   u_pi          +---------+   u    +------------------+ duty
                +-->(+)-->|  C(s) = Kp + Ki/s |--------------->|   sum   |------->| N(.) deadband    |---+
                    ^ -   |        (PI)       |                | u_pi +  |        | comp + saturation|   |
                    |     +-------------------+                |  u_ff   |        +------------------+   |
                    |                                          +---------+                               v
                    |                                                                          +-----------------+
                    |                                                                          |     G_m(s)      |
                    |                                                                          | motor + wheel   |
                    |                                                                          |  (duty -> w)    |
                    |                                                                          +-----------------+
                    |                                                                                   |
                    |                                                                                 w |
                    +------------------------------ w (encoder) <---------------------------------------+
```

PI on the speed error plus a **feedforward** term, then deadband compensation,
then output saturation:

```
e        = w_set - w_meas                 # speed error [rad/s]
i_acc   += Ki * e * dt                     # integral state
i_acc    = clamp(i_acc, -i_max, +i_max)    # anti-windup (see below)
u        = ff(w_set) + Kp * e + i_acc      # raw duty before deadband
duty     = deadband_compensate(u)
duty     = clamp(duty, -1, +1)            # output saturation (PWM/actuator limit)
```

### Feedforward

The motor model in [../../simulation/params.m](../../simulation/params.m) is
roughly linear from duty to speed, so a near-correct duty for a target speed is
known in advance:

```
ff(w_set) = w_set / w_noload          # w_noload = 333 * 2*pi/60 ~= 34.9 rad/s
```

The feedforward does most of the work; the PI only trims the residual error
(load, mismatch, battery sag). This keeps the PI gains small and the loop stable.

### Deadband compensation

Below ~10% duty the motor does not turn, so a small PI output would be wasted. Remap
a non-zero command up past the deadband (from
[../../simulation/MAPPING.md](../../simulation/MAPPING.md) section 2,
`deadband = 0.10`):

```
duty = sign(u) * (deadband + (1 - deadband) * |u|)   for u != 0
duty = 0                                              for u == 0
```

Measured on hardware with the Phase 4 `deadband` sweep: both motors start moving
at ~10% duty, so `deadband = 0.10` (was seeded at `0.05` from the sim).

### Anti-windup

Clamp the integral state `i_acc` to `[-i_max, +i_max]`. Additionally, when the
output saturates at `+/-1`, stop integrating in the direction that would push it
further into saturation (clamp / conditional integration). This prevents the
integrator from building up while the motor is maxed out and overshooting when
the setpoint drops.

## Measurement: per-tick wheel rate

The loop needs `w_meas` **every tick (200 Hz)**. The current telemetry fields
`velL`/`velR` are only computed once per second
([../../firmware/main/control.c](../../firmware/main/control.c) lines 98-110), so
they are too coarse for this loop.

Plan: add a small helper (e.g. `encoder_rate_radps(i, dt)`) that, per wheel,
deltas `encoder_count(i)` since its last call and converts to rad/s:

```
d_counts = count_now - count_prev
w_meas   = encoder_cps_to_radps(d_counts / dt)
```

At 200 Hz the per-tick count delta is small, so expect quantization noise; a
light first-order low-pass on `w_meas` (or a short moving average) may be needed.
This is noted here as a design consideration; the exact filter is decided during
bring-up against real encoder data.

## Sign conventions

- `+w_set` = forward; `+duty` = forward.
- Encoder signs are already normalized so `+counts = forward` for both wheels
  (`ENC_L_SIGN = +1`, `ENC_R_SIGN = -1` in
  [../../firmware/main/encoders.c](../../firmware/main/encoders.c)), so a positive
  error always means "spin this wheel forward faster". No per-wheel sign flips
  belong in this controller.

## Gains and tuning

Start conservative and tune one wheel at a time:

1. Set `Kp` small, `Ki` modest, no derivative term (`Kd = 0`).
2. Command a step in `w_set` (manually, see below) and watch `w_meas` track it.
3. Raise `Kp` until the response is brisk without oscillating, then add `Ki`
   until steady-state error is gone, keeping the integral clamp sane.
4. Tune the inner loop **tight** - its bandwidth must be well above the future
   balance loop so the cascade does not fight itself.
5. Repeat for the other wheel; the gains can differ slightly per motor.

Seed constants (refine on hardware): `w_noload = 34.9 rad/s` (from the sim),
`deadband = 0.10` (measured on hardware, Phase 4 deadband sweep).

## Telemetry and test plan

For bring-up and tuning, expose per wheel: `w_set`, `w_meas`, and `duty` (to be
added to `telemetry_t` / the JSON when the firmware is written -
[../../firmware/main/telemetry.h](../../firmware/main/telemetry.h)).

Bring-up procedure (before any balance loop exists):

1. Hold the robot off the ground (wheels free).
2. Drive a manual `w_set` over the existing WebSocket terminal (extend the motor
   command path to accept a speed target).
3. Confirm each wheel reaches the commanded `w_meas`, and that **both wheels reach
   the same speed for the same `w_set`** despite the motor mismatch - this is the
   whole point of the controller.
4. Step the setpoint up/down and check tracking, overshoot, and that the
   integrator does not wind up at saturation.
5. Record working gains back into this doc.
