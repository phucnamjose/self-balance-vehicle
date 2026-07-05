# Motor tuning

Pick the wheel-speed PI gains from the identified motor model and check the
result in the frequency domain (Nyquist + Bode, stability margins). The theory
(IMC pole-zero cancellation, the gain formulas, why the loop has finite margins)
is in [../../docs/theory/pi-tuning.md](../../docs/theory/pi-tuning.md); this page
is the hands-on script.

## Model

Each free-spinning wheel is first order from PWM duty `u` (−1..+1) to wheel
angular speed `ω` (rad/s); the PI closes the loop:

```
G(s) = K / (tau*s + 1)        C(s) = Kp + Ki/s
```

`K` and `tau` come from [../motors_identify](../motors_identify). The IMC rule
places the PI zero on the plant pole (`Ti = tau`) and leaves one knob, the
closed-loop time constant `tau_cl`:

```
Kp = tau / (K*tau_cl)         Ki = 1 / (K*tau_cl)
```

## Run

[`pi_tuning_analysis.m`](pi_tuning_analysis.m) takes your motor's `K` and `tau`
(and optionally `tau_cl`), falling back to the identified defaults
(`K = 34`, `tau = 0.19`, `tau_cl = tau/5`) when called with none:

```bash
cd experiments/motor_tuning
octave --eval "pi_tuning_analysis"                 # identified defaults
octave --eval "pi_tuning_analysis(34, 0.19)"       # your K, tau
octave --eval "pi_tuning_analysis(50, 0.12, 0.03)" # also pick tau_cl [s]
```

It prints `Kp`, `Ki`, the gain crossover `wc`, phase margin `PM` and gain margin
`GM`, and saves two figures:

- `pi-tuning-nyquist.png` — open-loop `L(jω)` vs the `−1` point
- `pi-tuning-bode.png` — magnitude/phase with the margins marked

Default parameters overwrite those base names (the figures used in the theory
doc). Custom parameters auto-suffix the filenames
(`pi-tuning-nyquist-K<..>_tau<..>_tcl<..>.png`) so a run never clobbers the
committed defaults; pass a 4th `tag` argument to force your own name, e.g.
`pi_tuning_analysis(50, 0.12, 0.03, 'fast')`.

## Things to watch

- **Margins, not gains** — retune by watching `PM`/`GM`, not by pushing `Kp`
  blindly. Targets: `PM > 45–60°`, `GM > 6 dB`.
- **`tau_cl` has a floor** — shrinking it raises `wc`, where the sample-and-hold
  delay eats phase margin. Faster is not free; re-run and check the margins.
- **Uses base Octave** — no `control` package needed; `L(jω)` is evaluated on a
  frequency grid by hand.
