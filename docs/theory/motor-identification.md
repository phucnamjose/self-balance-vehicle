# Motor Identification

Where the plant $G_m(s)$ (duty -> wheel speed) used by the
[wheel-speed-controller](wheel-speed-controller.md) comes from. We measure it on
the real robot instead of trusting the datasheet, then fit a simple model.

> Status: **done** for a first pass. The procedure and scripts live in
> [../../experiments/motors_identify/](../../experiments/motors_identify/)
> (`README.md`, `input.csv`, `motor_id.m`); this page is the theory behind them.

> Math below renders in GitHub and in Cursor's Markdown preview (KaTeX). If your
> viewer shows the raw `$...$`, use the Markdown preview.

## The model

Each wheel, free-spinning (no load), is modeled as **first order** from PWM duty
$u$ (in $-1..+1$) to wheel angular speed $\omega$ (rad/s):

$$
\tau\,\frac{d\omega}{dt} + \omega = K u
\qquad\Longleftrightarrow\qquad
G_m(s) = \frac{K}{\tau s + 1}
$$

- $K$ - steady-state gain [rad/s per unit duty] = no-load speed at full duty
- $\tau$ - mechanical time constant [s]

Two numbers per wheel are enough to design the inner speed loop: $K$ sets the
feedforward ($u_{ff} = 1/K$) and the loop DC gain, $\tau$ sets how fast the wheel
responds (hence the achievable bandwidth).

## Why first order (the physics)

A brushed DC motor driven at duty $u$ from supply voltage $V$ has armature
current $i = (uV - K_e\omega)/R$ (applied voltage minus back-EMF, over winding
resistance), and torque $T = K_t i$. With everything above the gearbox lumped
into a reflected wheel inertia $J$ and light viscous friction, the wheel obeys:

$$
J\,\frac{d\omega}{dt}
  = \frac{K_t\,(uV - K_e\omega)}{R} - (\text{friction})
  = \frac{K_t V}{R}\,u \;-\; \frac{K_t K_e}{R}\,\omega \;-\; (\text{friction})
$$

The electrical time constant $L/R$ is far faster than the mechanical one, so the
current settles "instantly" and the dynamics collapse to a single first-order
pole. Rearranging into the model form gives the parameter meanings:

$$
\tau = \frac{R J}{K_t K_e}\ \text{(electromechanical, independent of }V),
\qquad
K = \frac{V}{K_e}\ \text{(no-load speed per unit duty, }\propto V)
$$

This connects back to the torque-speed line in
[../../simulation/params.m](../../simulation/params.m)
($T(u,\omega) = u\,\tau_{stall} - (\tau_{stall}/\omega_{noload})\,\omega$): for a
free wheel,

$$
\omega_{noload} = K, \qquad
\tau_{stall} = \frac{J K}{\tau} \quad (J \approx \texttt{p.I\_wheel})
$$

## Estimating K and tau from a step response

Drive a sequence of constant-duty steps (both directions, several magnitudes)
and record $\omega$ at the control rate. Two independent estimates are used - they
should agree when the data is clean:

### Static gain + deadband

For each constant-duty segment, take the settled speed $\omega_{ss}$ (mean over
the tail of the segment). Fitting $|\omega_{ss}|$ vs $|u|$ gives:

- slope -> $K$
- x-intercept -> the **deadband** (the $|u|$ at which the wheel first turns; below
  it, static friction wins and $\omega = 0$)

### Dynamic (time constant), by least squares

Write the model as a linear regression in the measured signals:

$$
\frac{d\omega}{dt} = a\,\omega + b\,u + c
\qquad\text{with}\qquad
a = -\frac{1}{\tau}, \quad b = \frac{K}{\tau}
$$

Estimate $d\omega/dt$ by finite difference, stack $[\,\omega\ \ u\ \ 1\,]$ over
the **whole** run, and solve `[a b c] = [w u 1] \ dw` (ordinary least squares).
Then

$$
\tau = -\frac{1}{a}, \qquad K = -\frac{b}{a}
$$

The offset `c` absorbs a constant (Coulomb) friction bias. Doing it globally
over all samples averages out the encoder's velocity quantization (one count per
tick is ~0.95 rad/s), so a short zero-phase smoothing of $\omega$ before
differentiating is enough; no per-segment curve fitting is needed.

> **Discretization note.** This regression uses a forward-Euler derivative, not
> the exact zero-order-hold (ZOH) map. With $\Delta t = 5$ ms and $\tau \approx
> 0.2$ s the Euler bias on $\tau$ is ~1% - negligible here. The input *is* a true
> ZOH (playback holds each duty for its whole step), so if bias ever matters, fit
> the discrete ARX $\omega[k{+}1] = A_d\,\omega[k] + B_d\,u[k]$ and convert
> $a = \ln(A_d)/\Delta t$.

## What the model deliberately ignores

- **Nonlinear friction / stiction** beyond the single `deadband` + $c$ terms.
- **Load** - identification is done wheels-up; on the ground the effective $J$
  and friction rise, lowering bandwidth. Re-check gains under load.
- **Saturation** - the linear model holds between the deadband and full duty.

## Supply-voltage dependence

From the physics above, $K$ is **proportional to supply voltage** while $\tau$
is **not**. Over an 11.1-12.6 V battery, $K$ (and top speed) swing ~13%
end-to-end; $\tau$ stays put. Two ways to keep $G_m(s)$ valid as the battery
drains:

1. **Duty normalization** - scale the command by $V_{ref}/V_{batt}$ in firmware
   so the *effective* applied voltage is constant (removes the dependence).
2. **Robust design** - a closed loop tolerates a ~13% gain change easily; tune at
   the low-voltage worst case and verify stability at the high end.

Either way, identify at (or normalize to) the voltage you actually run at. Note
the deadband, expressed as a *duty*, also grows slightly as the battery sags
(a fixed breakaway voltage is a larger fraction of a smaller supply).

## Result feeds

- $\omega_{noload}$ (= $K$) and $\tau_{stall}$ -> update the motor block in
  [../../simulation/params.m](../../simulation/params.m).
- $K$, $\tau$, `deadband` -> feedforward, gains, and deadband compensation in the
  [wheel-speed-controller](wheel-speed-controller.md).
