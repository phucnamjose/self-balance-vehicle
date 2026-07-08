# Inverted pendulum figures

Generates the plots for
[../../docs/theory/inverted-pendulum.md](../../docs/theory/inverted-pendulum.md) -
the university-level theory of the inverted-pendulum-on-a-cart model that the
whole balance study rests on: the geometry, why the upright is unstable, the pole
picture, the phase-plane saddle, and why a linear model about upright is
legitimate.

Base Octave only (no toolboxes). The script **reuses the model** in
[../../simulation](../../simulation) - it `addpath`s that folder and calls
`params`, `plant_dynamics`, and `linearize` directly, so these figures can never
drift from the simulation. Change a parameter in `simulation/params.m` and every
figure here updates with it.

## Run

```bash
cd experiments/inverted_pendulum
octave --eval inverted_pendulum_plots
```

It writes five PNGs **into `docs/theory/`** (where the doc embeds them):

- `ip-schematic.png` — the cart-pendulum: coordinates ($x$, $\theta$, $l$) and
  the forces (gravity on the CoM, drive force $F$ on the base)
- `ip-fall.png` — open-loop ($F=0$): the upright runs away exponentially
  (time-to-double ~34 ms) while the same body hung down just oscillates
- `ip-poles.png` — $s$-plane poles: the inverted pendulum has a real pole in the
  right half-plane ($+20.57$ rad/s); the hanging version sits on the imaginary
  axis
- `ip-phase.png` — phase portrait ($\theta$ vs $\dot\theta$): upright is a
  **saddle** (unstable), hanging is a **centre** (stable), with the separatrix
  highlighted
- `ip-linearization.png` — why the linear model is valid near upright:
  $\sin\theta\approx\theta$ and linear-vs-nonlinear fall trajectories

## Parameters

There are none to set here - all physical/control values come from
[../../simulation/params.m](../../simulation/params.m). Edit the model there and
re-run.
