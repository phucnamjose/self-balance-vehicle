# Loop-rate figures

Generates the plots for
[../../docs/theory/loop-rates.md](../../docs/theory/loop-rates.md) - the
frequency-domain view of the whole control stack: which rate each part of the robot
runs at (PWM, IMU, control tick, cascade loops, telemetry) and why.

Base Octave only (no `control` package): every transfer function is evaluated on a
grid by hand, same as [../motor_tuning](../motor_tuning).

## Run

```bash
cd experiments/loop_rates
octave --eval loop_rate_plots
```

It writes four PNGs **into `docs/theory/`** (where the doc embeds them):

- `loop-rates-map.png` — the system frequency map: plant dynamics, loop crossovers,
  sampling/sensor rates, and the PWM carrier all on one log axis
- `loop-rates-cascade.png` — nested closed-loop bandwidths, showing the clean
  velocity/balance separation but the balance/wheel-speed overlap (the "squeeze")
- `loop-rates-delay.png` — the sample-and-hold phase penalty `-ω·T_d` vs frequency
  for 100 / 200 / 400 Hz, i.e. why 200 Hz is the sweet spot
- `loop-rates-antialias.png` — the IMU DLPF vs the Nyquist frequency: how
  above-Nyquist vibration is attenuated before it folds into the signal band

## Parameters

Key frequencies (unstable pole, wheel-loop crossover, DLPF, sample rate) are
hard-coded near the top of [`loop_rate_plots.m`](loop_rate_plots.m); edit them to
explore other designs. The plant/tuning numbers themselves come from
[../../simulation](../../simulation) (`linearize.m`) and [../motor_tuning](../motor_tuning).
