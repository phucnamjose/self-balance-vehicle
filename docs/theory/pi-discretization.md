# Discretizing the Wheel PI: from the s-Domain to a Sampled Loop

The wheel controller is *designed* in continuous time - $C(s) = K_p + K_i/s$,
tuned by pole-zero cancellation and checked on a Nyquist plot
([pi-tuning.md](pi-tuning.md)). A digital controller cannot evaluate that integral
continuously: it wakes at fixed sample instants, computes one number, and **holds**
it until the next instant. This note is the control-engineering principle for
turning the continuous PI (and its measurement filter) into a discrete-time law,
and for choosing a discretization that preserves the design. It is theory only -
the controller's structure and interface are in
[wheel-speed-controller.md](wheel-speed-controller.md).

> Math renders in GitHub and Cursor's Markdown preview (KaTeX). If you see raw
> `$...$`, use the preview. Rates and delays come from [loop-rates.md](loop-rates.md);
> gains from [pi-tuning.md](pi-tuning.md).

## The sampled-data setting

A digital loop differs from the continuous ideal in two ways:

- **Sampling.** The error $e(t)$ is only known at the tick instants
  $t_k = k\,\Delta t$, with sample period $\Delta t = 1/f_s$ (here
  $f_s = 200$ Hz, $\Delta t = 5$ ms).
- **Zero-order hold (ZOH).** The output $u[k]$ is applied and held **flat** across
  the whole interval $[t_k, t_{k+1})$, since the actuator is updated once per tick.

So the design task is: replace the continuous law by a **difference equation** -
an update computed once per tick from sampled values - that behaves like $C(s)$.

Split the PI into its two parts:

$$
u(t) = \underbrace{K_p\,e(t)}_{\text{memoryless}}
     + \underbrace{K_i\!\int_0^t e(\tau)\,d\tau}_{\text{has memory}}
$$

The proportional term is **instantaneous**: $u_P[k] = K_p\,e[k]$ needs no
discretization at all. All of the subtlety is in the integrator $1/s$ - the only
part with internal state - so "discretizing the PI" means "discretizing one
integral."

## Three ways to discretize the integrator

Over one tick the integral grows by the area of $e$ under that step,
$\int_{t_{k-1}}^{t_k} e\,d\tau$. The three standard rules just approximate that
sliver of area differently:

| Rule | Area estimate | Integral update $I[k] = I[k-1] + \dots$ | $s \to z$ map | Accuracy |
|------|---------------|-----------------------------------------|---------------|----------|
| **Forward** (left rectangle) | $\Delta t\,e[k-1]$ | $K_i\,\Delta t\,e[k-1]$ | $s = \dfrac{z-1}{\Delta t}$ | $O(\Delta t)$ |
| **Backward** (right rectangle) | $\Delta t\,e[k]$ | $K_i\,\Delta t\,e[k]$ | $s = \dfrac{z-1}{z\,\Delta t}$ | $O(\Delta t)$ |
| **Tustin** (trapezoid) | $\tfrac{\Delta t}{2}\big(e[k]+e[k-1]\big)$ | $K_i\,\tfrac{\Delta t}{2}\big(e[k]+e[k-1]\big)$ | $s = \dfrac{2}{\Delta t}\dfrac{z-1}{z+1}$ | $O(\Delta t^2)$ |

Each is just a substitution $s \to$ (some function of $z$) applied to $C(s)$.
Their properties differ in two ways that matter:

- **Accuracy.** Trapezoidal is one order better - it matches the frequency
  response most closely - and is the method of choice when discretizing a whole
  transfer function (e.g. a PID with a derivative filter).
- **Stability of the mapping.** Backward Euler and Tustin map the entire stable
  left half-plane *inside* the unit circle, so they can never turn a stable design
  unstable. Forward Euler maps it into a shifted disk that pokes outside the unit
  circle, so with a large enough gain-times-step it *can* destabilize a design that
  was stable in continuous time.

## The discrete PI

Take the simplest rule, **forward Euler**, and apply it to the integrator. With
$I[k]$ the integral state entering tick $k$:

$$
\boxed{\,u[k] = K_p\,e[k] + I[k], \qquad I[k{+}1] = I[k] + K_i\,\Delta t\,e[k]\,}
$$

i.e. keep a running sum of the error, scaled by $K_i\,\Delta t$, and add it to the
proportional term. In the $z$-domain the controller is

$$
C(z) = K_p + \frac{K_i\,\Delta t}{z - 1}
$$

whose pole at $z = 1$ is the unit-circle image of the continuous integrator's pole
at $s = 0$ - a discrete integrator, exactly as intended.

This is the **positional form**: the full output is rebuilt each tick from
$K_p e$ plus the stored integral. An equivalent **incremental (velocity) form**
propagates only the change $\Delta u[k] = u[k] - u[k-1] = K_p\,(e[k]-e[k-1]) +
K_i\,\Delta t\,e[k]$; it never stores an absolute integral (handy for bumpless
transfer and some anti-windup schemes) but must be seeded carefully. The two are
algebraically identical.

### Why the choice barely matters at a high sample rate

Forward Euler's conditional stability sounds alarming, but it is a non-issue when
the loop is sampled far faster than its own dynamics - the usual design rule
$f_s \gtrsim 20\,f_c$ ([loop-rates.md](loop-rates.md)). Two small numbers show why,
using the wheel loop's crossover $\omega_c = 25.8$ rad/s:

- **Heavy oversampling.** $\omega_c\,\Delta t = 25.8 \times 0.005 = 0.13$ rad - the
  loop is ~50x faster than its own bandwidth, the regime where *all three* rules
  converge to the continuous design and to each other.
- **Tiny per-tick increment.** $K_i\,\Delta t \approx 0.77 \times 0.005 = 0.0039$
  per unit error-second - a slow, smooth accumulation, nowhere near a step that
  could ring or diverge.

The whole practical difference between forward and backward Euler is *which* error
sample drives the update - i.e. **one sample of delay**, $\Delta t = 5$ ms. At the
crossover that is $\omega_c\,\Delta t = 7.4^\circ$ of extra phase, and it is
**already in the margin budget**: the tuning in [pi-tuning.md](pi-tuning.md) lumps
the ZOH + compute latency into a transport delay $T_d \approx 1.5\,\Delta t$ and
still lands $67^\circ$ of phase margin. So at 200 Hz the choice of integration rule
is a rounding error on the design - the reason a simple running sum is legitimate.

## Discretizing the measurement filter

A first-order low-pass often precedes the error (to tame sensor quantization noise
on the measured speed):

$$
F(s) = \frac{1}{\tau_f\,s + 1}
$$

Discretizing this is the same exercise, but here the natural choice is **backward
Euler** ($s \to (z-1)/(z\,\Delta t)$). Starting from $\tau_f\,\dot y + y = x$ with
$\dot y \approx (y[k]-y[k-1])/\Delta t$:

$$
\tau_f\frac{y[k]-y[k-1]}{\Delta t} + y[k] = x[k]
\;\Longrightarrow\;
y[k] = y[k-1] + \alpha\,\big(x[k] - y[k-1]\big),
\qquad
\alpha = \frac{\Delta t}{\tau_f + \Delta t}
$$

the familiar **exponential smoother**. Backward Euler is deliberate for a filter:
it is *unconditionally stable*, and its coefficient $\alpha = \Delta t/(\tau_f +
\Delta t)$ lies in $(0,1)$ for **any** sample rate, so the smoother can never
overshoot or ring no matter how $\Delta t$ is chosen. (Forward Euler on the same
filter gives $\alpha = \Delta t/\tau_f$, which exceeds 1 - and rings - if the tick
is ever slow relative to $\tau_f$.) The principle: use the unconditionally stable
rule where robustness matters (the filter), the simple rule where it is cheap and
safe (the slow integrator).

## Anti-windup is naturally discrete

Whenever the actuator saturates, the integrator keeps summing an error it can no
longer act on, then has to unwind before the output leaves the limit - the classic
overshoot after a large setpoint change. On a sampled loop the cure is a per-tick
decision: **conditional integration** - each tick, if the output is already pinned
at its limit *in the same direction as the error*, skip that tick's integral
update (freeze the running sum, do not reset it); otherwise integrate normally. A
hard clamp on the integral state is kept as a second line of defence. This is
simply the discrete-time statement of "stop integrating while saturated," and it
composes cleanly with the difference equation above because the integral is updated
one tick at a time anyway.

## The plant side: exact ZOH discretization

To check stability or simulate the closed loop in discrete time you also need the
plant sampled. Unlike the controller, the first-order motor
$G(s) = K/(\tau s + 1)$ has an **exact** discrete equivalent under a zero-order
hold - no approximation - precisely because the input really is held flat each tick:

$$
\omega[k{+}1] = a_d\,\omega[k] + b_d\,u[k], \qquad
a_d = e^{-\Delta t/\tau}, \quad b_d = K\,(1 - a_d)
$$

Pairing an **exact-ZOH plant** with an **Euler-discretized controller** is the
standard, adequate combination: the plant dynamics we care about are captured
exactly, and the controller error is negligible at $\omega_c\,\Delta t \ll 1$.

## Verifying the discretization preserved the design

The principle to check is that the sampled loop reproduces the continuous
closed-loop response ($\tau_{cl} = \tau/5$ from [pi-tuning.md](pi-tuning.md)). A
minimal step simulation - discrete PI (forward-Euler integral, conditional
anti-windup) around the exact-ZOH plant, with the backward-Euler measurement
filter - does exactly that:

```octave
K = 34; tau = 0.19; dt = 1/200;          % plant + tick
Kp = tau/(K*tau/5); Ki = 1/(K*tau/5);    % IMC gains, tau_cl = tau/5 (pi-tuning.md)
ad = exp(-dt/tau); bd = K*(1-ad);        % exact ZOH plant
tau_f = 0.02; a = dt/(tau_f+dt);         % backward-Euler measurement LPF
w = 0; wf = 0; I = 0; wsp = 10; N = 200; W = zeros(1,N);
for k = 1:N
  wf = wf + a*(w - wf);                   % filter the measurement
  e  = wsp - wf;
  u  = Kp*e + I;                          % P + stored integral
  us = max(-1, min(1, u));
  if u == us, I = I + Ki*e*dt; end        % forward-Euler integral w/ anti-windup
  w  = ad*w + bd*us;  W(k) = w;           % plant advances one tick
end
plot((1:N)*dt, W); grid on; xlabel('t [s]'); ylabel('\omega [rad/s]');
```

The step should settle at the setpoint with the first-order shape of the
continuous design, to within the one-sample sampling delay - the practical
confirmation that discretization changed the *implementation*, not the *behaviour*.
