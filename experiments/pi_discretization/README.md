# PI discretization figures

Generates the plots for
[../../docs/theory/pi-discretization.md](../../docs/theory/pi-discretization.md) -
how the continuous wheel-speed PI $C(s) = K_p + K_i/s$ becomes a difference
equation the 200 Hz digital loop can run, with the frequency-domain comparison of
the discretization rules (forward Euler, backward Euler, Tustin).

Base Octave only (no `control` package): every transfer function is evaluated on a
grid by hand, same as [../motor_tuning](../motor_tuning).

## Run

```bash
cd experiments/pi_discretization
octave --eval discretization_plots
```

It writes four PNGs **into `docs/theory/`** (where the doc embeds them):

- `pi-disc-sampling.png` — sampling + zero-order hold in the time domain
- `pi-disc-splane.png` — where the s-plane `jω` axis lands in the z-plane under
  each rule (the stability picture: forward pokes outside the unit circle,
  backward stays inside, Tustin lands exactly on it)
- `pi-disc-bode.png` — Bode of the integrator `Ki/s` vs its three discrete
  approximations (they agree below the crossover, fan out near Nyquist)
- `pi-disc-step.png` — closed-loop step: continuous design vs the discrete (ZOH)
  loop, showing the behaviour is preserved

## Parameters

Plant/controller values are hard-coded to the identified motor and IMC gains
(`K = 34`, `tau = 0.19`, `tau_cl = tau/5`, `dt = 1/200`); edit the top of
[`discretization_plots.m`](discretization_plots.m) to explore other rates or gains.
