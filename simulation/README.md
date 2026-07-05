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
| 6 | `sim_discrete.m` | Realistic 200 Hz loop: sensors, filter, motor model |
| 7 | `lqr_design.m`, `lqr_gain.m` | LQR / state-feedback (self-contained Riccati solver); compare against PID |
| 8 | `MAPPING.md` | Convert tuned gains + states to firmware units |

The plotting scripts also write a `*.png` of their result next to the script.

## Running

From this folder in an Octave prompt (or `octave script.m`):

```octave
p = params();        % load parameters
sim_openloop          % run a step (scripts auto-load params)
```

Each script is standalone: it loads `params()`, runs, prints a summary, and
plots. Parameters marked "MEASURE" in `params.m` are estimates - refine them
from the real robot to make the sim trustworthy.
