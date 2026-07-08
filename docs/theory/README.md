# Control Design

How the self-balancing vehicle is controlled. We build the controllers **one at
a time** (same step-by-step style as [../ROADMAP.md](../ROADMAP.md)); each gets
its own design doc here before any firmware is written.

## The problem

The two drive motors (GB37-520) are not identical: the same PWM duty produces
slightly different wheel speeds because of unit-to-unit variation, gearbox
friction, and a per-motor deadband. Driven open-loop (which is what the firmware
does today - `firmware/main/control.c` just applies the web-terminal command to
each motor), this makes the robot **veer** and lets one wheel outrun the other.
A balance controller sitting directly on top of two mismatched actuators has to
fight that mismatch on every tick.

## The fix: cascade control

Wrap each motor in an inner **wheel speed controller** that closes the loop on
the encoder. The outer loops then command a *speed* instead of a raw duty, and
the inner loop guarantees the wheel actually reaches that speed - absorbing motor
mismatch, deadband, and back-EMF so the outer loops see two well-behaved,
identical wheels.

Continuous-domain (s-domain) block diagram of the full intended cascade. Every
block is a Laplace transfer function, `(+)` is a summing junction (`-` marks the
subtracted/measured input), and the lines back to the junctions are feedback.
Sensors (encoders, IMU) are idealized as unity feedback. The two inner per-wheel
loops are drawn collapsed into the plant block here (`w_set_L`, `w_set_R` from the
mixer); their internal structure is in
[wheel-speed-controller.md](wheel-speed-controller.md):

```text
   theta_ref=0    e_theta   +--------------+ w_common
   ----->(+)-------------->|  C_theta(s)  |----------+
          ^ -              | balance_ctrl |          |
          |                +--------------+          |
          |                                          v
          |                                      +-------+   w_set_L
          |                                      | mixer |------------>+
          |                                      |       |   w_set_R   |
          |                                      +-------+------------>|
          |                                          ^                 v
          |                                          |      +-------------------------+
          |                                          |      |  wheel-speed loops L,R  |
          |                                          |      |   + robot body = G(s)   |
          |                                          |      +-------------------------+
          |   psi_ref    e_psi  +-----------+ w_diff |         |              |
          |   ----->(+)-------->|  C_psi(s) |--------+         | psi          | theta
          |          ^ -        |  yaw_ctrl |                  |              |
          |          |          +-----------+                  |              |
          |          +------------------ psi <-----------------+              |
          |                                                                   |
          +-------------------------- theta <---------------------------------+
```

Mixer: `w_set_L = w_common + w_diff`, `w_set_R = w_common - w_diff`. The wheels
drive the body, so the same plant that produces the wheel speeds also produces the
tilt (`theta`) and heading (`psi`) fed back to the outer loops.

Controller transfer functions (continuous-time):

- `C_w(s) = Kp + Ki/s` - inner wheel-speed PI (plus a feedforward path; see
  [wheel-speed-controller.md](wheel-speed-controller.md)). **This step.**
- `C_theta(s) = Kp + Ki/s + Kd*s` - balance PID (`Kd` fed from the gyro). Later.
- `C_psi(s)` - yaw/heading controller. Later.

Everything except `C_w(s)` (`wheel_speed_ctrl`) is **not built yet** - the diagram
shows the intended full structure so the naming and interfaces stay consistent as
we add each piece. Working from the inside out, the inner speed loop is the
foundation the rest of the stack stands on, so it comes first.

> The diagram is the continuous-time idealization used for design and tuning. On
> the robot it runs discretely - inner speed + balance loops at 200 Hz, the outer
> velocity/yaw loops sub-rated to ~50 Hz. The whole rate stack (PWM, IMU, control
> tick, cascade sub-rates, telemetry) is designed in [loop-rates.md](loop-rates.md),
> and how each continuous law $C(s)$ becomes the difference equations that actually
> run is in [pi-discretization.md](pi-discretization.md).

## Naming scheme

All controllers use a consistent `*_ctrl` name, organized by loop layer. This
keeps future files and docs predictable.

| Name | Role | Output | Status |
|------|------|--------|--------|
| `wheel_speed_ctrl` | Inner per-wheel angular-speed loop (one instance per wheel) | motor duty `[-1, 1]` | **this step** |
| `balance_ctrl` | Standing / upright tilt loop | common wheel speed `w_common` [rad/s] | **in firmware (`balance_pid`), gains untuned** |
| `velocity_ctrl` | Forward speed / position-hold | a tilt or speed target | later |
| `yaw_ctrl` | Heading / turn-rate; sets the wheel differential | `w_diff` [rad/s] | later |
| `mixer` | Combine common + differential into per-wheel setpoints | `w_set_L`, `w_set_R` | later |
| `pid` | Shared PID/PI building block used by the controllers above | - | later |

Convention: a **common** quantity drives both wheels the same (forward motion); a
**differential** quantity drives them oppositely (turning). The mixer is just
`w_set_L = w_common + w_diff`, `w_set_R = w_common - w_diff`.

## Build order

Each controller links to its own design doc as it is written.

0. [motor-identification.md](motor-identification.md) - measure the plant
   `G_m(s)` (duty -> wheel speed) on hardware. **(done)**
1. [wheel-speed-controller.md](wheel-speed-controller.md) - inner per-wheel
   angular-speed loop. **(current step)**
2. [balance-controller.md](balance-controller.md) - standing/tilt loop
   (`balance_ctrl`). *(in firmware as `balance_pid`; gains need hardware tuning)*
   Rests on two foundations:
   [inverted-pendulum.md](inverted-pendulum.md) - the plant model (why upright is
   unstable, its poles, the 34 ms time-to-double, controllability); and
   [angle-estimation.md](angle-estimation.md) - fusing the accelerometer + gyro
   (complementary / Kalman filter) into the $\theta$, $\dot\theta$ it feeds back.
3. `mixer` + `yaw_ctrl` - drive straight and steer. *(later)*
4. `velocity_ctrl` - forward drive / station-keeping. *(later)*
