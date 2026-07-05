# Mapping the Simulation to the Firmware

The simulation thinks in SI units (meters, radians, Newtons). The firmware
thinks in what the hardware actually produces: **encoder counts**, **degrees**,
**deg/s**, and a **motor command in [-1, 1]**. This note converts between them so
the gains tuned in Octave can be dropped into the ESP32 control loop
([../firmware/main/app_main.c](../firmware/main/app_main.c)) in Phase 9.

All numbers below come from the current `params.m`. If you change parameters,
re-run the helper at the bottom to regenerate them.

## 1. State conversions

| Sim quantity | Unit | Firmware source | Conversion |
|--------------|------|-----------------|------------|
| `theta` (tilt) | rad | IMU pitch (accel + gyro) | `theta_deg = theta_rad * 180/pi` |
| `omega` (tilt rate) | rad/s | gyro axis, `gx`/`gy`/`gz` | already in deg/s: `omega_dps = omega_rad * 180/pi` |
| `pos` (cart) | m | encoders `posL`,`posR` | `pos_m = avg(posL,posR) / 6271.2` |
| `vel` (cart) | m/s | encoders `velL`,`velR` | `vel_ms = avg(velL,velR) / 6271.2` |

Where **6271.2 counts/m** = `counts_per_wheel_rev / (2*pi*r_wheel)` =
`1320 / (2*pi*0.0335)`. (One wheel revolution = 1320 counts = `2*pi*r` meters.)

Cart position/velocity use the **average** of the two wheels (the common,
forward motion). The IMU gives the tilt; which gyro axis and which accel
components form the pitch depends on how the MPU6050 is mounted - lock that down
during Phase 6 wiring.

Tilt estimate in firmware (the complementary filter from `sim_discrete.m`), run
every control tick (`dt = 1/CONTROL_HZ = 0.005 s`):

```c
// theta_acc from accel (small-angle), omega from the gyro pitch axis (deg/s):
float theta_acc = atan2f(accel_horizontal, accel_vertical) * 180.0f/M_PI;
theta_deg = 0.995f*(theta_deg + omega_dps*0.005f) + 0.005f*theta_acc;
```

## 2. Controller output -> motor command

The controllers output a **force F [N]**. The motor command is a duty in
[-1, 1] (`s_motor_cmd[]` in the firmware). Convert, then apply the two
real-world compensations the sim showed were necessary:

```
duty_raw = F / F_stall                       ; F_stall = 29.27 N (both motors, w=0)
duty_ff  = duty_raw + (K_bemf * vel_ms)/F_stall   ; back-EMF feedforward, K_bemf=25.06 N/(m/s)
duty     = deadband_compensate(duty_ff)      ; remap [0,1] -> [deadband, 1]
duty     = clamp(duty, -1, +1)
```

- **F_stall = 29.27 N** is the force both motors make at zero speed; it is how
  duty maps to force. (Top cart speed is `w_noload*r ~= 1.17 m/s`.)
- **Back-EMF feedforward** uses the encoder speed `vel_ms`; without it the cart
  is too sluggish to catch the body (this was what made `sim_discrete.m` topple
  until it was added).
- **Deadband compensation**: below ~10% duty the motor does not move, so remap
  `sign(u)*(deadband + (1-deadband)*|u|)`. Measured in Phase 4 (deadband sweep):
  both motors ~10%, so `p.motor.deadband = 0.10`.

## 3. PID gains (Steps 5-6) in firmware units

Sim PID (force units): `F = Kp*theta + Ki*Int(theta) + Kd*omega`,
with `Kp=45, Ki=15, Kd=5` (theta in rad, omega in rad/s).

If you compute the PID directly in **degrees / deg-per-second** and output
**duty**, fold `1/F_stall` and `pi/180` into the gains:

| Gain | Sim value (N per rad) | Firmware value (duty per deg) |
|------|----------------------|-------------------------------|
| Kp | 45 | **0.02683** duty / deg |
| Ki | 15 | **0.00894** duty / (deg*s) |
| Kd | 5  | **0.00298** duty / (deg/s) |

Sanity check: a 5 deg tilt -> `0.02683*5 = 0.134` = ~13% duty, which matches the
duty trace in `sim_discrete.png`.

## 4. LQR gain (Step 7) in firmware units

Sim LQR: `F = -K*x`, `K = [-10.00, -14.02, -96.03, -13.52]` for
`x = [pos(m), vel(m/s), theta(rad), omega(rad/s)]`, `Q=diag([5 1 200 5])`,
`R=0.05`.

Computing the duty directly from firmware-native measurements:

```
duty = -( Kc*avg(posL,posR) + Kvc*avg(velL,velR) + Kth*theta_deg + Kom*omega_dps )
```

| Term | Multiplies | Firmware coefficient |
|------|-----------|----------------------|
| `Kc`  | encoder counts        | **5.447e-05** duty / count |
| `Kvc` | encoder counts/s      | **7.635e-05** duty / (count/s) |
| `Kth` | tilt [deg]            | **0.05725** duty / deg |
| `Kom` | tilt rate [deg/s]     | **0.00806** duty / (deg/s) |

(These already include the `-1/F_stall`, `1/6271.2`, and `pi/180` factors and the
sign of `u = -K*x`.) Still apply back-EMF feedforward + deadband compensation
from section 2.

## 5. Before trusting any of this on hardware

1. **Measure the real parameters** (`params.m` "MEASURE" items): wheel radius,
   body mass, CoM height, body height. The gains scale with these; the current
   numbers are educated estimates.
2. **Calibrate the gyro bias** (Phase 9). The sim assumes ~0.1 deg/s residual
   after calibration; raw bias is much larger and will make the robot lean.
3. **Fix the sign conventions**: positive command = forward, encoder counts up =
   forward, positive tilt = forward (see the GB37 doc). Flip a motor's wires or
   a sign in firmware if not.
4. **Measure the deadband** (Phase 4 motor test) and the actual top speed; update
   `p.motor.deadband` and `p.motor.w_noload`. Deadband done: both motors ~10%,
   `p.motor.deadband = 0.10`; top speed / `w_noload` still to measure.
5. **Start gentle**: lower gains first, hold the robot, watch the telemetry
   (`gx/gy/gz`, `posL/posR`) before letting it stand on its own.

## Regenerate these numbers

From this folder in Octave:

```octave
p = params(); [A,B] = linearize(p);
cpm   = p.counts_per_wheel_rev/(2*pi*p.r_wheel);
Fst   = p.n_wheels*p.motor.tau_stall/p.r_wheel;
Kbemf = p.n_wheels*p.motor.tau_stall/(p.motor.w_noload*p.r_wheel^2);
K = lqr_gain(A,B,diag([5 1 200 5]),0.05);
printf('counts/m=%.1f  F_stall=%.2f  K_bemf=%.2f\n', cpm, Fst, Kbemf);
```
