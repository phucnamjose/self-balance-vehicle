# Wheel Speed Controller

The inner loop of the cascade (see [README.md](README.md)). One instance per
wheel regulates that wheel's **angular speed** to a commanded setpoint, so the
two non-identical motors behave identically to whatever sits above them.

## Purpose

Regulate one wheel's angular speed to a setpoint, independently of the other
wheel. This rejects the per-motor differences (unit variation, gearbox friction,
deadband, back-EMF) so an upper loop can command a single common speed and trust
that **both wheels actually reach it** - the robot drives straight instead of
veering.

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
then output saturation. The pseudocode below is the continuous law written as a
per-tick update; how that update is *derived* from $C(s) = K_p + K_i/s$ (which
discretization rule, and why it is safe at 500 Hz) is in
[pi-discretization.md](pi-discretization.md):

```
e        = w_set - w_meas                 # speed error [rad/s]
i_acc   += Ki * e * dt                     # integral state
i_acc    = clamp(i_acc, -i_max, +i_max)    # anti-windup (see below)
u        = ff(w_set) + Kp * e + i_acc      # raw duty before deadband
duty     = deadband_compensate(u)
duty     = clamp(duty, -1, +1)            # output saturation (PWM/actuator limit)
```

### Feedforward

The motor model (`G_m(s) = K/(tau*s+1)`, measured per
[motor-identification.md](motor-identification.md) and seeded in
[../../simulation/params.m](../../simulation/params.m)) is roughly linear from
duty to speed, so a near-correct duty for a target speed is known in advance:

```
u_ff(w_set) = w_set / K               # K = steady-state gain [rad/s per duty]
```

`K` is the per-wheel steady-state gain (no-load speed at full duty) from motor
identification ([../../experiments/motors_identify/motor_id.m](../../experiments/motors_identify/motor_id.m)):
`K_L ~= 34.36`, `K_R ~= 32.18 rad/s per duty`.

The feedforward does most of the work; the PI only trims the residual error
(load, mismatch, battery sag). This keeps the PI gains small and the loop stable.
It can be toggled at runtime (`ff on|off`, or the *feedforward* checkbox in the
web UI) to compare against pure PI.

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

The loop needs `w_meas` **every tick (500 Hz)**. The current telemetry fields
`velL`/`velR` are only computed once per second
([../../firmware/main/control.c](../../firmware/main/control.c) lines 98-110), so
they are too coarse for this loop.

Plan: add a small helper (e.g. `encoder_rate_radps(i, dt)`) that, per wheel,
deltas `encoder_count(i)` since its last call and converts to rad/s:

```
d_counts = count_now - count_prev
w_meas   = encoder_cps_to_radps(d_counts / dt)
```

At 500 Hz the per-tick count delta is small, so expect quantization noise; a
light first-order low-pass on `w_meas` (or a short moving average) may be needed.
This is noted here as a design consideration; the exact filter is decided during
bring-up against real encoder data.

### As built: two smoothing stages, and why `tau_f` ships at 0

Bring-up ended with **two** low-pass stages in series on the speed feedback, and
the interaction between them was the dominant source of step **overshoot** -
worth recording as a lesson:

1. **Sliding-window velocity** (`VEL_WIN = 5` in `control.c`): speed is a finite
   difference of encoder counts over the last ~10 ms. This *is* the velocity
   estimator, and it sets the **quantization floor**. At 1320 counts/rev one
   count is `2π/1320 ≈ 0.0048 rad`, so a single 2 ms tick would quantize speed to
   `~2.4 rad/s`, while the 10 ms window brings that to `~0.48 rad/s`. The window
   is mandatory; its width is a resolution choice, and it costs ~4-5 ms of delay.
2. **Measurement LPF `tau_f`** (`wheel_pi.c`): a first-order filter on that
   *already-windowed* speed, feeding the PI. This is extra smoothing on a signal
   the window has largely cleaned up.

Running both stacks their lag (`~4-5 ms + 20 ms ≈ 25 ms`), which erodes phase
margin and showed up as **13-50% overshoot** on speed steps. Because the window
already handles quantization, the fix was simply **`tau_f = 0`** (filter off):
the overshoot vanished with no downside beyond a slightly noisier duty. Rule of
thumb: keep *one* primary smoother (the window) and treat `tau_f` as a small
live-tunable trim (`~0.003-0.005 s`) only if the duty chatters - don't tune two
overlapping low-passes against the same noise. The empirical closed-loop check is
in [../../experiments/closed-loop_identify/](../../experiments/closed-loop_identify/)
(`tau_cl ≈ 0.023-0.026 s`, overdamped, unity DC gain).

## Sign conventions

- `+w_set` = forward; `+duty` = forward.
- Encoder signs are already normalized so `+counts = forward` for both wheels
  (`ENC_L_SIGN = +1`, `ENC_R_SIGN = -1` in
  [../../firmware/main/encoders.c](../../firmware/main/encoders.c)), so a positive
  error always means "spin this wheel forward faster". No per-wheel sign flips
  belong in this controller.

## Gains and tuning

For the analytic starting point - pick `Kp`, `Ki` directly from the identified
`K`, `tau` by pole-zero cancellation, with a Nyquist stability check - see
[pi-tuning.md](pi-tuning.md). The hardware procedure below then fine-tunes it.

Start conservative and tune one wheel at a time:

1. Set `Kp` small, `Ki` modest, no derivative term (`Kd = 0`).
2. Command a step in `w_set` (manually, see below) and watch `w_meas` track it.
3. Raise `Kp` until the response is brisk without oscillating, then add `Ki`
   until steady-state error is gone, keeping the integral clamp sane.
4. Tune the inner loop **tight** - its bandwidth must be well above the future
   balance loop so the cascade does not fight itself.
5. Repeat for the other wheel; the gains can differ slightly per motor.

### Default gains (in firmware)

The values tuned on hardware and compiled in as the defaults
([../../firmware/main/wheel_pi.c](../../firmware/main/wheel_pi.c),
overridable live with `gains <l|r|both> <kp> <ki>`). The two motors differ, so
the gains are **per wheel**:

| Wheel | `Kp` | `Ki` | `Ti = Kp/Ki` | Feedforward `K` [rad/s per duty] |
|-------|------|------|--------------|----------------------------------|
| Left  | 0.1455 | 0.6737 | 0.216 s | 34.36 |
| Right | 0.1554 | 0.8265 | 0.188 s | 32.18 |

The integral time `Ti` sits on the order of the motor `tau` (~0.19 s), as the
pole-zero cancellation in [pi-tuning.md](pi-tuning.md) predicts; the small
per-wheel spread is the measured motor mismatch this loop exists to absorb. The
feedforward `K` is each motor's identified steady-state gain
([motor-identification.md](motor-identification.md)).

Other compiled defaults (`wheel_pi.c`): measurement LPF `tau_f = 0` (**off** -
see [below](#as-built-two-smoothing-stages-and-why-tau_f-ships-at-0);
live-tunable with `tauf <s>`), output cap `±0.95`, integral clamp `±0.95`, brake
cap `0.4`, deadband compensation **off** by default (neutral `0.02`, floor `0.10`
when on), feedforward **on**.

Seed constants (for reference): `w_noload = 34.9 rad/s` (from the sim),
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
5. Record working gains back into this doc (done - see *Default gains* above).
