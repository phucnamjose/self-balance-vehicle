# Balance-Bot Simulation (Octave)

An offline study of the self-balancing vehicle as a **balance-only inverted
pendulum on a cart**, used to understand the physics and design + tune a
controller before running it on the real robot (Phase 8 in
[../docs/ROADMAP.md](../docs/ROADMAP.md)).

Built and run with **GNU Octave** (tested on 8.4.0). Steps 1-6 need only base
Octave; the symbolic derivation (Step 2) needs the `symbolic` package.

## The model

The body above the wheel axle is a pendulum that wants to fall; the wheels are a
cart we can drive to keep the body upright.

```
        body CoM (mass m_body, height l)
            \
             \  theta  (0 = upright)
              \ |
   ------------ O ------------   <- wheel axle (the "cart", mass M)
        ()              ()       <- wheels, radius r
   =============================  ground, cart position = pos
```

State everywhere: `x = [pos; vel; theta; omega]`
(cart position [m], cart velocity [m/s], body tilt from upright [rad], tilt rate
[rad/s]). `theta > 0` = tipped forward. All SI units inside the sim.

## Files (build order)

| Step | File | What it does |
|------|------|--------------|
| 1 | `params.m` | All physical + control parameters (edit me) |
| 2 | `eom_derive.m`, `plant_dynamics.m` | Symbolic equations of motion -> numeric state derivative |
| 3 | `sim_openloop.m` | Integrate the uncontrolled plant; watch it fall |
| 4 | `linearize.m` | Linearize about upright; eigenvalues + controllability |
| 5 | `sim_closedloop_pid.m` | Balance it with a tuned PID; plot recovery |
| 6 | `sim_discrete.m` | Realistic 500 Hz loop: sensors, filter, motor model (force-based) |
| 7 | `lqr_design.m`, `lqr_gain.m` | LQR / state-feedback (self-contained Riccati solver); compare against PID |
| 8 | `MAPPING.md` | Convert tuned gains + states to firmware units |
| 9 | `wheel_pi_sim.m`, `balance_pid_sim.m` | Faithful ports of the firmware controllers (`wheel_pi.c`, `balance_pid.c`) |
| 9 | `sim_cascade.m` | Multi-rate (motor 500 / imu 250 / balance 250 Hz) **firmware cascade**: balance PID -> wheel-speed PI -> motor -> plant |
| 9 | `tune_cascade.m` | Sweep balance-PID gains over `sim_cascade` and score for a clean balance |

The plotting scripts also write a `*.png` of their result next to the script.

## Two ways the balance loop is closed

- **Force-based** (`sim_closedloop_pid.m`, `sim_discrete.m`): the PID computes a cart
  **force** directly. Simplest to reason about, but it is *not* how the firmware runs.
- **Cascade** (`sim_cascade.m`): mirrors the firmware exactly - the balance PID outputs a
  common **wheel-speed** command `w_common`, and an inner wheel-speed PI turns that into
  motor duty. `balance_pid_sim.m` / `wheel_pi_sim.m` are line-for-line ports of the C, so
  gains tuned here transfer to the robot.

  A key consequence, which the cascade makes visible: with a wheel-*speed* inner loop the
  base *acceleration* is the derivative of the command, so the balance gain **roles rotate** -
  `Ki` is the restoring *stiffness* (and must satisfy `r*Ki > g`, i.e. `Ki > g/r ~= 302`), `Kp`
  is *damping*, and `Kd` adds effective base inertia - which now *slows/under-damps* the catch,
  so **keep it small** (opposite of a force-PID's derivative term).

  **The inner loop's speed and lag matter a lot.** `params.m` models the identified actuator
  lag (`p.motor.tau_e ~= 0.19 s`); `sim_cascade.m` drives the inner loop with the *actual*
  firmware `wheel_pi` gains, the measurement low-pass **OFF** (`tau_f = 0`, as shipped) and
  feedforward on - so the sim inner loop matches the robot's identified ~25 ms closed-loop
  *tracking*, not an idealized fast one. On this honest plant (multi-rate 500/250/250):
  - A robust balance is a broad, flat optimum around **`Kp~=45, Ki~=450, Kd~=3`** (`Ki` well
    above the `g/r~=302` stiffness floor, small `Kd`) - `tune_cascade.m` searches and scores
    this. These are the firmware seed gains; tune on the robot from here.
  - With the fast `tau_f=0` inner loop the duty follows hot commands faithfully, so over-large
    gains (especially `Kd`) chatter the duty and the cart acceleration corrupts the tilt
    estimate - the sweep favors the lowest gains that still hold. Tilt-only still drifts
    (~30 cm/8 s); a velocity/position outer loop is the next layer.

## Running

From this folder in an Octave prompt (or `octave script.m`):

```octave
p = params();        % load parameters
sim_openloop          % run a step (scripts auto-load params)
```

Each script is standalone: it loads `params()`, runs, prints a summary, and
plots. Parameters marked "MEASURE" in `params.m` are estimates - refine them
from the real robot to make the sim trustworthy.
