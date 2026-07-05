# Motor identification

Estimate a simple per-wheel motor model from a recorded step response, so the
simulation and controllers use numbers measured on the real robot. The theory
(derivation, estimator, voltage dependence) is in
[../../docs/theory/motor-identification.md](../../docs/theory/motor-identification.md);
this page is the hands-on procedure.

## Model

Each free-spinning wheel is treated as a first-order system from PWM duty
`u` (−1..+1) to wheel angular speed `ω` (rad/s):

```
tau * dω/dt + ω = K * u      <=>      G(s) = K / (tau*s + 1)
```

- `K` — steady-state gain [rad/s per unit duty] = no-load speed at full duty
- `tau` — mechanical time constant [s]

This maps onto the torque–speed line in `simulation/params.m`
(`tau(u,ω) = u·tau_stall − (tau_stall/w_noload)·ω`): for a free wheel
`w_noload = K` and `tau_stall = I_wheel · K / tau`.

## 1. Record a step response

1. Lift the wheels off the ground (free-spin, no load) and power at the nominal
   battery voltage you will actually run.
2. In the web app: `control start`, then select **TEST_MOTORS** (`exp motors`)
   so the loop applies the raw duty with no estimator/controller.
3. **Motor script** → load [`input.csv`](input.csv) (one step per line
   `dur,mL,mR`: hold that duty for `dur` seconds). It plays each step in turn,
   then parks — you get a notice when it finishes.
4. **Data log** → topic `motors` → **Record to file** just before playing, and
   stop when it finishes. This saves `t, velL, velR, velL_sp, velR_sp, mL, mR`.

The step list should cover both directions and a range of duties, holding each
long enough (≥ ~1 s given `tau ≈ 0.2 s`) for the speed to settle.

## 2. Fit the parameters

Run the Octave script on the recording (rename it to `motors.csv` for the
default, or pass the path):

```bash
cd experiments/motors_identify
octave --eval "motor_id('motors_YYYYMMDD_HHMMSS.csv')"
```

It prints `K`, `tau` and the deadband per wheel, maps them to
`w_noload`/`tau_stall`, and saves a `<csv-name>.png` overlaying the measured
speed with the identified model. Two independent estimates are produced:

- **static** — steady-state speed of each constant-duty segment → gain + deadband
- **dynamic** — one global least-squares fit of `dω/dt = a·ω + b·u + c` over the
  whole run → `tau = −1/a`, `K = −b/a` (the offset `c` captures static friction)

The reported `R²` is the fit quality; a low value means that wheel's data is
noisy or the wheel/encoder misbehaved — re-check it before trusting the numbers.

## Things to watch

- **Sign** — a negative `K` means that wheel's encoder/motor sign is flipped
  relative to the duty. Fine for modeling; fix the sign for closed-loop control.
- **Deadband** — to resolve it, include small duties (0.02, 0.04, …); if the
  smallest tested duty already spins the wheel, the deadband can't be measured.
- **Supply voltage** — `K` is proportional to the supply voltage while `tau` is
  not. Identify at the voltage you run at, or normalise the duty by battery
  voltage in firmware so `G(s)` stays fixed as the battery drains.
