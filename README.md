# Self-Balancing Robot — LQR Control & 3D Simulation

Optimal control design and simulation of an inverted-pendulum-on-wheels
system, built up from a classical cart-pole benchmark to a two-wheeled
self-balancing robot, with a fully custom 3D visualization pipeline.

MATLAB · Control System Toolbox (`lqr`, `ctrb`) · ODE45 nonlinear simulation

---

## Overview

This project designs and validates a **Linear-Quadratic Regulator (LQR)**
controller for balancing an inverted pendulum on a moving base, in two
stages:

1. **Cart-pole benchmark** (`cartpend.m`, `cartpole_lqr.m`) — the classic
   textbook inverted pendulum on a linearly sliding cart. Used to validate
   the modeling and control pipeline against known analytic results before
   extending to a harder, less standard system.
2. **Two-wheeled self-balancing robot** (`two_wheeled_robot_simulation.m`)
   — the same balancing problem, but the base moves by *wheel rolling*
   instead of sliding on a frictionless rail, and the input is motor
   **torque** instead of a direct force. This is closer to a real
   hardware platform.

Both systems are controlled with LQR designed from a linearized model,
then validated in closed loop against the full **nonlinear** dynamics via
`ode45` — not just the linear approximation the controller was designed
from.

## Methodology

**Nonlinear dynamics.** Derived from first principles via the
Euler-Lagrange method for a point-mass pendulum on a cart/rolling base,
with `theta = 0` defined as the upright (unstable) equilibrium. See the
derivation and full state convention documented directly in `cartpend.m`.

**Linearization, cross-checked two ways.** The controller is designed
from a linear model obtained via **numerical (finite-difference)
linearization** of the nonlinear dynamics about the upright equilibrium.
This is cross-checked at runtime against the closed-form analytic
linearization (`cartpole_lqr.m`, Section 3) — if the two diverge by more
than numerical tolerance, that's flagged immediately as a modeling bug,
rather than discovered later from an unstable simulation.

**Controllability.** Verified via `rank(ctrb(A,B)) == 4` before any
controller design is attempted.

**LQR weight selection, justified not guessed.** State order is
`[position, velocity, angle, angular velocity]`. The angle state is
weighted an order of magnitude above position/velocity in `Q`, because a
large angle error means the pendulum is actually falling — a failure
mode — while a large position error is comparatively harmless and
recoverable. `R` (control effort weight) is chosen via an explicit
sweep across `[0.1, 1, 10, 100]`, showing the effort/speed trade-off
directly rather than asserting a single number is correct by fiat.

**Validation, not just visualization.** Every run reports closed-loop
eigenvalues (explicit stability check), max control effort, max angle
deviation, and final tracking error — numbers, not just "the plot looks
stable."

## Key result: recovery from a 50° initial disturbance

The two-wheeled robot controller was stress-tested from a **50° initial
tilt** (roughly 8x a typical "gentle nudge" test) using an **event-based
ODE solver** (`ode45` + custom `balance_event` function) that stops the
simulation the instant the robot is genuinely balanced — angle, angular
velocity, and forward velocity all simultaneously within tight
tolerances — rather than running a fixed, arbitrarily-chosen duration.

| Metric | Result |
|---|---|
| Initial tilt | 50° |
| Time to balance | **4.22 s** |
| Final angle error | 0.117° |
| Final position error | 2 mm (target 0.5 m) |
| Peak control torque | 10.57 N·m |

**Honest caveat, not a gap I'm hiding:** LQR is designed from a
small-angle linearization (`sin(theta) ≈ theta`). At 50° that
approximation is well outside its normal validity range, yet the
controller still recovers — a genuinely interesting robustness result.
The peak torque of ~10.6 N·m is large relative to what a small
hobby-scale robot's actuators could realistically deliver, which is
flagged as a known next step (see **Future Work**) rather than glossed
over — the controller is mathematically correct; whether it's physically
buildable as-is is a separate, open question.

## Two implementation approaches

The two-wheeled robot control law was implemented **twice**, in two
different environments, as its own point of validation:

1. **Script-based** (`src/two_wheeled_robot_simulation.m`) — nonlinear
   dynamics as a MATLAB function, `ode45` for integration, LQR gains
   applied directly in code. This is the main implementation, with the
   event-based 50° recovery test and 3D visualization described above.
2. **Block-diagram (Simulink)** (`src/simulink/`) — the same dynamics
   and LQR feedback law built as a Simulink model (`model.slx`): an
   `fcn` block for the nonlinear dynamics, an integrator, a `-K*u`
   feedback gain block closing the loop, and a scope. Visualized with a
   2D stick-figure animation (`animate_robot.m`) driven by the logged
   `out.sim_data`. See `src/simulink/README_SIMULINK.md` for how to run
   it.

Having both demonstrates the same controller works whether it's
expressed as code or as a block diagram — and shows familiarity with
both MATLAB scripting and Simulink modeling workflows.

## 3D Visualization

`visualize_robot_3d.m` renders the simulated trajectory as a fully 3D
scene — not a 2D stick figure — built with plain MATLAB graphics
(`patch`, `hgtransform`, two-point lighting, checkerboard floor for depth
cues, a motion trail tracing the pendulum tip, and a live HUD showing
position/angle/torque). No Simscape/Simulink dependency: it runs
anywhere plain MATLAB does, reading directly from the `ode45` output.

An MP4/AVI export of the animation is included in `media/` (see below).

## Repository structure

```
src/
  cartpend.m                    # Cart-pole nonlinear dynamics (benchmark validation)
  cartpole_lqr.m                # Cart-pole LQR design, sim, verification
  two_wheeled_robot_simulation.m # Two-wheeled robot: LQR design + nonlinear sim + balance-event
  visualize_robot_3d.m          # 3D CAD-style visualization + video export
  state_estimator_kalman.m      # Kalman filter: noisy sensors, unmeasured states inferred
  kalman_simulation.m           # Kalman filter + realistic torque saturation, 50-degree test
  mpc_vs_lqr.m                  # LQR vs PID vs MPC controller comparison
  simulink/
    model.slx                   # Block-diagram implementation of the same control law
    animate_robot.m             # 2D animation driven by the Simulink model's logged output
    README_SIMULINK.md          # How the Simulink model is wired, how to run it
media/
  robot_balance_recovery.mp4    # Rendered animation of the 50° recovery (script-based)
README.md
```

## How to run

1. Open `src/two_wheeled_robot_simulation.m` in MATLAB (Online or
   desktop) and run it. This computes `t`, `x`, `K`, `x_ref`, `params`
   in the workspace and prints the balance-time / performance metrics.
2. Run `src/visualize_robot_3d.m` in the same session — it reuses those
   workspace variables to render (and optionally export) the 3D
   animation.
3. `src/cartpole_lqr.m` + `src/cartpend.m` can be run independently to
   reproduce the cart-pole benchmark validation.

Requires MATLAB with Control System Toolbox (`lqr`, `ctrb`).

## Realistic sensing, actuator limits, and controller comparison

Everything above assumes perfect state feedback and unlimited torque —
neither is true for a real robot. Three further experiments push past
that:

### State estimation (Kalman filter)

`src/state_estimator_kalman.m` replaces perfect state feedback with a
realistic sensor model: position (encoder) and angle (IMU) are measured
with real noise; velocity and angular rate are **never directly
measured** and must be inferred by a Kalman filter. First attempt
diverged (RMS angle error 384°) because the estimator started "blind"
(assuming upright/stationary with no basis) against a system whose
open-loop instability is fast (~11.8 rad/s) — a real race the estimator
lost. Fixed by seeding the estimator from an actual first sensor
reading and increasing process-noise weighting on the unmeasured
states (forcing the filter to trust its own model-propagation of
velocity/rate less). Final result: RMS position error 6mm, RMS angle
error 0.89°.

### Actuator saturation

`src/kalman_simulation.m` adds a hard ±10 N·m torque limit on top of
the Kalman-filtered LQR loop, tested at 50°. This produced a genuine,
informative failure the first time: the *unsaturated* peak torque
needed for 50° recovery was 10.565 N·m — capping at 10 N·m removed
just enough margin that the robot didn't recover slower, it **failed
completely** (angle ran to -25,847°, ~72 full rotations). This
revealed how thin the actual torque margin is at that disturbance
level, not a bug. After the same estimator retune above, the
same 50°/±10N·m test **does** recover (final angle 0.98°).

### Controller comparison: LQR vs PID vs MPC

`src/mpc_vs_lqr.m` runs the same nonlinear plant, same 50° disturbance,
same ±10 N·m limit through three different controllers for a
like-for-like comparison (true-state feedback here, isolating
controller differences from the estimation work above):

| Controller | Final angle | Final pos. error | Notes |
|---|---|---|---|
| LQR | -4.33° | 0.081m | Clean, smooth recovery |
| PID (cascade) | -4.27° | 0.153m | Recovers, but chatters against the torque limit noticeably more than LQR |
| MPC (custom, via `quadprog`) | does not reliably recover | — | See below |

**A real bug was found and fixed during this comparison**: the initial
PID implementation diverged monotonically to 10,000+ degrees. Root
cause was an inverted error sign in the inner angle loop — comparing
against the (working) LQR gain's sign convention on the theta/thetadot
terms revealed the PID was computing positive feedback on an
already-unstable system. Fixed by correcting the error definition and
re-verified with the outer position loop disabled first, then
re-enabled, to isolate the fix before recomposing the full cascade.

**MPC — built from scratch, not the toolbox `mpc()` object** — is the
one controller that plans explicitly around the ±10 N·m constraint
instead of clipping an unconstrained law after the fact, which is the
real theoretical case for expecting it to handle saturation better.
In practice, hand-tuning its cost weights did not converge to reliable
recovery: an LQR-matched weighting under-reacted and diverged; a
heavily angle-weighted retune kept the pole bounded but abandoned
position tracking (119m error); a middle-ground retune diverged again.
The likely root cause is numerical conditioning: with a 40-step horizon
and this system's discrete growth rate (~1.125×/step, compounding to
~112× by the end of the horizon), the prediction matrices span a wide
enough numerical range that small weight changes produce
disproportionate, hard-to-predict behavior. A shorter, better-conditioned
horizon was identified as the most likely fix but not yet tried — see
Future Work. Documented honestly as an open problem rather than
tuned to look like it works.

## Future work

- **Wheel rotational inertia (`Iw`)** is currently defined as a parameter
  but not incorporated into the dynamics — the model treats wheels as
  inertia-free. Including it would make the torque numbers above more
  physically realistic.
- **MPC horizon conditioning.** The custom linear MPC controller does
  not currently recover reliably (see comparison section above). The
  most likely fix — a shorter prediction horizon for better numerical
  conditioning, with the LQR-matched cost weights — was identified but
  not yet tried.
- Port the cart-pole model to Simulink/Simscape Multibody for a
  block-diagram / physical-modeling demonstration alongside the
  script-based version here.
