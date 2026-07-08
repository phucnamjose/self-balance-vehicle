# Balance controller figures

Generates the plots for
[../../docs/theory/balance-controller.md](../../docs/theory/balance-controller.md) -
the university-level theory of the standing/tilt loop that keeps the robot
upright: what it must do, the tilt PID that does it, *why* feedback can stabilize
the unstable plant, and why full-state (LQR) feedback is needed to also hold
position.

Base Octave only (no toolboxes). The script **reuses the model + design code** in
[../../simulation](../../simulation) - it `addpath`s that folder and calls
`params`, `plant_dynamics`, `linearize`, and `lqr_gain` directly, so these
figures can never drift from the simulation. Change a parameter in
`simulation/params.m` and every figure here updates with it.

## Run

```bash
cd experiments/balance_controller
octave --eval balance_controller_plots
```

It writes four PNGs **into `docs/theory/`** (where the doc embeds them):

- `balance-recovery.png` — closed-loop recovery from an initial $4^\circ$ tilt
  plus a mid-run disturbance kick, next to the open-loop fall it is fighting;
  bottom panel shows the drive effort staying inside the motor ceiling
- `balance-pid-vs-lqr.png` — angle-only PID balances but the cart drifts
  (~44 cm); full-state LQR balances **and** returns home
- `balance-poles.png` — $s$-plane: the open-loop pole at $+20.6$ rad/s is dragged
  across the imaginary axis into the left half-plane by the LQR gain (one fast
  closed-loop pole at $-466$ is off-scale)
- `balance-threshold.png` — the rightmost closed-loop pole of the tilt sub-loop
  vs the P-gain $K_p$, crossing zero at $K_p\approx 9.6$ N/rad — the "$K_p$ must
  beat gravity" stabilization condition

## Parameters

The physical/control values come from
[../../simulation/params.m](../../simulation/params.m); the PID gains
($K_p=45,\ K_i=15,\ K_d=5$) and LQR weights ($Q=\mathrm{diag}(5,1,200,5)$,
$R=0.05$) mirror `simulation/sim_closedloop_pid.m` and `simulation/lqr_design.m`.
Edit those and re-run.
