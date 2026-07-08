# Robot mechanics illustration

Generates the labelled mechanics figure for
[../../docs/hardware/robot-mechanics.md](../../docs/hardware/robot-mechanics.md) -
a to-scale side + front view of the robot with its dimensions (wheel Ø65 mm,
track 190 mm, height 120 mm) and the body-fixed MPU6050 axes (X front, Y left,
Z up).

Base Octave only (a plain 2-D vector drawing, no toolboxes).

## Run

```bash
cd experiments/robot_mechanics
octave --eval robot_mechanics_plot
```

It writes one PNG **into `docs/hardware/`**:

- `robot-mechanics.png` — side view (height, wheel Ø, axle height, CoM arm $l$,
  X/Z axes) and front view (track $W$, wheel Ø, Y/Z axes)

## Parameters

Dimensions are at the top of
[`robot_mechanics_plot.m`](robot_mechanics_plot.m) (`D`, `W`, `H`, and an
illustrative CoM height `l`). Edit them to match your build. The real physical
values also live in [../../simulation/params.m](../../simulation/params.m).
