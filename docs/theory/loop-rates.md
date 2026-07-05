# Loop Rates and Sampling Frequencies

One place that fixes **every rate in the stack** and says why - from the 10 kHz
motor PWM carrier at the top down to the 2 Hz telemetry frames at the bottom, with
the 200 Hz control tick in the middle that everything else hangs off. This is the
timing counterpart to the cascade in [README.md](README.md) and the loop-shaping
in [pi-tuning.md](pi-tuning.md).

> Math renders in GitHub and Cursor's Markdown preview (KaTeX). If you see raw
> `$...$`, use the preview.

## Three rules the whole design follows

1. **One master clock, integer sub-rates.** A single hardware timer ticks at
   `CONTROL_HZ` = 200 Hz
   ([../../firmware/main/telemetry.h](../../firmware/main/telemetry.h)); every
   slower activity (outer loops, telemetry) runs on an **integer division** of it.
   No second free-running loop, so there are no beat frequencies between
   asynchronous loops and every rate is a clean fraction of 200 Hz.
2. **Separation of timescales.** In a cascade each inner loop must look
   ~*instantaneous* to the loop wrapped around it, and each outer loop must look
   ~*quasi-static* (slowly varying) to the loop inside it. Rule of thumb: a factor
   of ~$3$-$5$ between successive loop **bandwidths** (crossover frequencies). Note
   this is about *bandwidth*, not sample rate - two loops can share the 200 Hz tick
   as long as their crossovers are separated.
3. **Sample fast, filter early.** Close a loop at $\omega_s \gtrsim 20\,\omega_c$
   (sample rate at least ~20x its crossover) so the sample-and-hold delay barely
   dents the phase margin and the discrete design behaves like the continuous one.
   Put every anti-alias / carrier frequency where it belongs: sensor bandwidth
   **below** the Nyquist of its read rate, PWM carrier **far above** the control
   rate.

## The plant timescales we must respect

The rates below are not free choices - they are pinned by how fast the physics
moves. Two numbers dominate.

**The upright is unstable and fast.** From
[../../simulation/linearize.m](../../simulation/linearize.m) (numbers from the
current [../../simulation/params.m](../../simulation/params.m)) the open-loop
poles are

$$
s \approx \{\,0,\; -0.01,\; -11.31,\; +11.19\,\}\ \text{rad/s}
$$

The one in the right half-plane, $+11.19$ rad/s, is the body tipping over: a small
tilt grows like $e^{11.19\,t}$, i.e. **time-to-double $\approx 62$ ms**
($\approx 1.8$ Hz). Everything in the balance path - sense, estimate, compute,
actuate - has to happen many times inside that 62 ms or the robot falls.

**The wheels are sluggish by comparison.** Each motor is first order with
$\tau \approx 0.19$ s ([motor-identification.md](motor-identification.md)), a pole
at $1/\tau \approx 5.3$ rad/s (0.84 Hz). The inner wheel-speed loop is tuned to a
crossover $\omega_c = 25.8$ rad/s (4.1 Hz) with $67^\circ$ phase margin
([pi-tuning.md](pi-tuning.md)).

Laid out on one frequency axis (rad/s), slowest dynamics on the left:

| Frequency [rad/s] | [Hz] | What it is |
|-------------------|------|------------|
| ~1-3 | ~0.2-0.5 | outer velocity / position loop crossover (target) |
| 5.3 | 0.84 | open-loop wheel pole $1/\tau$ |
| **11.19** | **1.78** | **unstable body pole** (time-to-double 62 ms) |
| ~20-28 | ~3-4.5 | balance loop crossover (must sit **above** the unstable pole) |
| 25.8 | 4.1 | inner wheel-speed loop crossover (tuned) |
| 276 | 44 | IMU DLPF cutoff (sensor bandwidth) |
| 628 | 100 | control-loop **Nyquist** ($\omega_s/2$ at 200 Hz) |
| 1257 | 200 | control sample rate $\omega_s$ |
| ~6.3e4 | 1e4 | motor PWM carrier |

The design job is to slot the digital rates around those physical ones so rule 2
(separation) and rule 3 (sampling) both hold.

## The control tick: why 200 Hz

`CONTROL_HZ = 200` is the master rate; the whole real-time path (read IMU ->
estimate tilt -> run the loops -> write PWM) executes once per 5 ms tick
([../../firmware/main/control.c](../../firmware/main/control.c)).

**It is fast enough for the unstable pole.** Over one tick the tilt grows by
$e^{11.19\cdot 0.005} = e^{0.056} \approx 1.06$ - about 6% per tick, so the loop
gets ~11 correction opportunities per doubling time. `sim_discrete.m` balances the
robot at exactly this rate with an honest sensor + motor model, which is the
practical proof.

**The sampling delay is affordable.** A zero-order hold plus one-tick compute
latency costs a transport delay of about

$$
T_d \approx 1.5\,\Delta t = 7.5\ \text{ms at 200 Hz},
$$

whose phase lag $-\omega\,T_d$ grows with frequency:

| At crossover | $\omega_c$ | delay phase $-\omega_c T_d$ |
|--------------|-----------|-----------------------------|
| wheel-speed loop | 25.8 rad/s | $-11^\circ$ |
| balance loop | ~22 rad/s | $-9^\circ$ |

Both are small change against a ~$45$-$70^\circ$ phase-margin budget - see the
identical $-11^\circ$ term in the [pi-tuning.md](pi-tuning.md) Nyquist analysis.

**Why not slower, why not faster.** There is a genuine sweet spot:

- **Slower (e.g. 100 Hz)** doubles $T_d$ to 15 ms. At a ~25 rad/s crossover that
  is $-22^\circ$ of phase, and with the unstable pole so close to that crossover
  the balance margin gets thin. 100 Hz is the floor, not a comfortable choice.
- **Faster (e.g. 400 Hz)** halves $T_d$ and buys back phase (worth ~$+10^\circ$ at
  a ~49 rad/s crossover - see the cascade section), *but* it worsens **encoder
  velocity quantization**: one count in a tick is
  $\tfrac{2\pi}{1320}/\Delta t$ rad/s $= 0.95$ rad/s at 200 Hz and doubles to
  ~1.9 rad/s at 400 Hz, which then needs heavier filtering (more phase lag - the
  $\tau_f \approx 8$ ms measurement low-pass in [pi-tuning.md](pi-tuning.md)),
  partly giving the phase back.

So **200 Hz is the baseline** (delay small, quantization tolerable, sim-validated).
If hardware bring-up shows the balance/inner cascade is too tight (next section),
**400 Hz is the first knob to turn** - it is a one-line change and the rest of the
ladder scales with it.

## Cascade rates and the tight-separation caveat

The intended stack, inner to outer ([README.md](README.md)):

| Loop | Runs at | Divisor of tick | Crossover target | Separation check |
|------|---------|-----------------|------------------|------------------|
| `wheel_speed_ctrl` (inner) | 200 Hz | ÷1 | 25.8 rad/s | fastest closed loop |
| `balance_ctrl` | 200 Hz | ÷1 | ~20-28 rad/s | **> unstable pole 11.2** |
| `yaw_ctrl` | 50 Hz | ÷4 | ~2-5 rad/s | decoupled (differential) |
| `velocity_ctrl` | 50 Hz | ÷4 | ~1-3 rad/s | below balance |

**The caveat.** For a clean cascade the inner wheel-speed loop (25.8 rad/s) should
sit well above the balance loop - but the balance loop must itself sit above the
unstable pole (11.2 rad/s) to stabilize it, which pushes its crossover up toward
~20-28 rad/s. Inner and balance crossovers then **overlap**, so the wheel loop is
*not* comfortably "instantaneous" to the balance loop. This is inherent to a small,
twitchy balancer (short body -> fast unstable pole), and it drives two choices:

1. **Push the inner loop tighter.** Retune the wheel-speed loop to
   $\tau_{cl}=\tau/10$, moving its crossover to $\omega_c \approx 49$ rad/s (the
   ideal $1/\tau_{cl}=53$, pulled down by the delay + measurement filter). Phase
   margin drops to ~$48^\circ$ at 200 Hz (still healthy); this restores roughly a
   2x separation over a ~25 rad/s balance loop. At 400 Hz the same crossover keeps
   ~$58^\circ$ - the concrete payoff of the higher rate.
2. **Sub-rate only the genuinely slow loops.** Velocity and yaw crossovers are
   ~1-5 rad/s, so 50 Hz (÷4) is >60x their bandwidth - plenty by rule 3 - while
   the ÷4 keeps them comfortably below the balance loop (rule 2) and saves CPU.
   Balance stays on the full 200 Hz tick because it lives right next to the
   unstable pole and cannot be slowed.

Firmware realization: the 200 Hz control task runs the inner + balance loops every
tick and the outer loops on a `tick % 4 == 0` sub-schedule (one code path, one
timer).

## Sensor rates and anti-aliasing

The MPU6050 ([../../firmware/main/imu.c](../../firmware/main/imu.c)) is configured:

- **Internal sample rate 1 kHz** (`SMPLRT_DIV = 0`) - it oversamples, then we
  decimate by reading once per 200 Hz tick.
- **Digital low-pass ~44 Hz** (`CONFIG = 0x03`, ~44 Hz accel / ~42 Hz gyro). This
  is the anti-alias filter for our 200 Hz read: 44 Hz sits below the 100 Hz
  Nyquist with ~2.3x margin, and well above the ~5 Hz band the balance loop
  actually uses, so the signal passes with negligible phase lag
  ($\arctan(5/42)\approx 7^\circ$ at 5 Hz) while structural vibration above ~100 Hz
  is attenuated before it can alias down.
- If hardware shows high-frequency vibration leaking in, drop the DLPF to ~21 Hz
  (`CONFIG = 0x04`) - still >4x the signal band. Do **not** raise it near 100 Hz.

**I2C bus 400 kHz** (`I2C_FREQ_HZ`): the 14-byte accel/temp/gyro block plus
addressing is ~20 bytes x 9 bits / 400 kHz $\approx 0.45$ ms, a ~9% slice of the
5 ms budget - fast enough to read every tick, which is why the estimator and
control share the 200 Hz rate instead of splitting.

## Actuator carrier: PWM at 10 kHz

Motor PWM runs at 10 kHz ([../../firmware/main/motors.c](../../firmware/main/motors.c),
`MOTOR_PWM_FREQ_HZ`), the XY-160D driver's ceiling. Placement:

- **Far above the mechanics.** The wheel bandwidth is ~5 rad/s (0.84 Hz); at
  10 kHz the motor sees a smooth average voltage (duty), not a switching square
  wave - the winding inductance filters the ripple current.
- **50x the control rate.** Each 5 ms tick spans 50 PWM periods, so the commanded
  duty is effectively a continuous actuator to the loop.
- **At the top of the audible edge.** 10 kHz can whine; the code comment notes to
  lower it if it whines or heats. It must stay well above the control rate, so
  don't drop it below a few kHz.

## Telemetry and reporting

Logging is deliberately decoupled from control. The loop writes **one sample per
tick (full 200 Hz)** into a double buffer; a batch of `SAMPLES_PER_BATCH =
CONTROL_HZ/2 = 100` samples (0.5 s) is handed to the reporter, which emits **~2
frames per second** ([../../firmware/main/control.c](../../firmware/main/control.c),
[../../firmware/main/telemetry.h](../../firmware/main/telemetry.h)). So we transmit
at a network-friendly 2 Hz but never lose resolution - each frame carries the full
200 Hz history. The per-tick period is checked against $\pm40\%$ limits
(`DT_MAX/MIN_WARN_US`) so timing jitter surfaces as a warning instead of silently
corrupting the loop.

## Summary: every rate and where it lives

| Rate | Value | ÷ of tick | Set in | Rationale |
|------|-------|-----------|--------|-----------|
| Motor PWM carrier | 10 kHz | x50 | `motors.c` `MOTOR_PWM_FREQ_HZ` | above mechanics + audible edge, ≤ driver ceiling |
| IMU internal ODR | 1 kHz | x5 | `imu.c` `SMPLRT_DIV=0` | oversample before decimation |
| I2C bus clock | 400 kHz | - | `imu.c` `I2C_FREQ_HZ` | read IMU block in ~0.45 ms ≪ tick |
| Control master tick | 200 Hz | x1 | `telemetry.h` `CONTROL_HZ` | ~11 updates / fall doubling; $T_d=7.5$ ms |
| Inner wheel-speed loop | 200 Hz | ÷1 | (firmware TBD) | crossover 25.8 rad/s (tighten to ~49) |
| Balance loop | 200 Hz | ÷1 | (firmware TBD) | crossover above unstable 11.2 rad/s |
| Yaw / velocity loops | 50 Hz | ÷4 | (firmware TBD) | slow, decoupled; separation + CPU |
| IMU DLPF cutoff | ~44 Hz | - | `imu.c` `CONFIG=0x03` | anti-alias < 100 Hz Nyquist |
| Telemetry frames | ~2 Hz | ÷100 | `telemetry.h` `SAMPLES_PER_BATCH` | network-friendly; full 200 Hz per frame |

Digital rates are all integer fractions of the 200 Hz tick, so changing
`CONTROL_HZ` rescales the loop rates and telemetry batch together (rule 1); the
PWM and IMU rates are independent hardware carriers and stay put.

## Reproduce the plant numbers

From `simulation/` in Octave (unstable pole, doubling time) and
`experiments/motor_tuning/` (inner-loop crossover + margins vs. $\tau_{cl}$ and
rate):

```bash
cd simulation
octave --eval "linearize"                          # open-loop poles; RHP pole + time-to-double
cd ../experiments/motor_tuning
octave --eval "pi_tuning_analysis(34, 0.19)"       # tau_cl=tau/5 -> wc=25.8, PM 67
octave --eval "pi_tuning_analysis(34, 0.19, 0.019)"# tau_cl=tau/10 -> wc~49, PM ~48
```
