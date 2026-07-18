# Closed-loop identification (inner wheel-speed PI)

Identify the *closed loop* of the inner wheel controller — the transfer from the
speed **setpoint** to the **measured** wheel speed, `T(s) = ω / ω_sp` — from a
recorded step/staircase response. This is the loop the outer balance controller
sees, so its time constant and overshoot set what the balance design can assume.
The open-loop motor model (duty → speed) is identified separately in
[../motors_identify](../motors_identify).

## Model

The wheel PI is tuned by IMC pole–zero cancellation, so the design target is a
first-order closed loop:

```
tau_cl * dω/dt + ω = K_cl * ω_sp      <=>      T(s) = K_cl / (tau_cl*s + 1)
```

- `K_cl` — steady-state tracking gain (want ≈ 1)
- `tau_cl` — closed-loop time constant [s] (design ≈ open-loop `tau` / 5)

The script fits this, and also an ARX(2,2) second-order model, so any resonance
or extra lag beyond first order shows up as `ωn`/`ζ` (or two real time
constants when overdamped).

## 1. Record a step response

1. Wheels off the ground (free-spin) at the battery voltage you run at.
2. Web app: `control start`, then **TEST_MOTOR_CONTROLLERS** (`exp motor-ctrl`)
   so the inner wheel PI runs but the estimator/balance loop stays off.
3. **Motors controller → speed script** → load a `dur,vL,vR` staircase (rad/s
   setpoints), e.g. [`input_whell_pi.csv`](input_whell_pi.csv): steps of 2 s up
   5→25 rad/s and back down. Hold each step ≥ ~1 s so the speed settles.
4. **Data log → topic `motors` → Record to file** just before playing; stop when
   it finishes. In this mode the `motors` topic streams at the full loop rate
   (500 Hz) and columns are
   `t, velL, velR, velL_sp, velR_sp, mL, mR, rawL, rawR`.

## 2. Fit the model

```bash
cd experiments/closed-loop_identify
octave --eval "closed_loop_id('motors_YYYYMMDD_HHMMSS.csv')"
```

It prints, per wheel:

- **1st order** — global least-squares `dω/dt = a·ω + b·ω_sp + c` →
  `tau_cl = −1/a`, `K_cl = −b/a`, with the fit `R²`.
- **2nd order** — ARX(2,2) least squares → continuous `ωn`, `ζ` (or two real
  `tau`), DC gain, and the *model's* unit-step overshoot / rise / settle.
- **data step** — overshoot, rise and settle measured straight off each
  setpoint step in the recording, plus a per-step breakdown.

It also saves `<csv-name>.png` overlaying the measured speed, the setpoint, and
both models.

## What the recording shows

On the current gains the linear fit is excellent and well damped:

- `K_cl ≈ 1.00`, `tau_cl ≈ 0.030–0.032 s` (R² ≈ 0.997), i.e. the loop tracks
  with ~30 ms and, as a linear system, **no overshoot** (ARX comes out
  overdamped).

But the raw steps overshoot a lot, and the overshoot **shrinks with speed**:

```
overshoot by step:  0->5: 43%  5->10: 49%  10->15: 41%  15->20: 18%  20->25: 9%
```

A single linear model cannot produce amplitude-dependent overshoot, so this is a
**nonlinear** effect, not a feedback-gain problem: from low speed the wheel has
to break static friction/deadband, the integrator + feedforward push hard, and
the wheel lurches past the setpoint. Once it is already spinning (20→25) the
breakaway is gone and the overshoot nearly vanishes — matching the linear fit.
Levers: reduce the proportional kick (setpoint weighting, `b≈0.3`), slew-limit
the setpoint, or compensate the deadband; raising damping/`tau_cl` alone will
not fix it because the linear loop is already well damped.

## Things to watch

- **Excitation** — use several step sizes in both directions (a staircase) so
  the least-squares fit sees enough dynamics; a single step barely constrains a
  2nd-order model.
- **Velocity quantization** — measured speed is quantized (~1 rad/s/count), so a
  2 % settling band on a 5 rad/s step is below the noise floor. The script
  floors the settling band at ~1 rad/s; for tight settle numbers use larger
  steps or a finer-resolution speed.
- **Setpoint saturation** — setpoints are clamped to ±29 rad/s in firmware
  (`WHEEL_PI_WSET_MAX`); keep the script inside that range or the "step" the
  loop actually sees is smaller than commanded.
