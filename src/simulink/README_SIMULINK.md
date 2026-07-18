# Simulink Implementation

`model.slx` (add your file here) implements the same LQR-controlled
two-wheeled balancing robot as `two_wheeled_robot_simulation.m`, but as a
**block diagram** instead of a script:

- `fcn` block — nonlinear dynamics (same equations as `robot_dynamics`
  in `two_wheeled_robot_simulation.m`), takes state `x` and control `u`,
  outputs `dx`
- `1/s` (integrator) — integrates `dx` to get the state `x`
- `-K*u` gain block — the LQR feedback law, closing the loop back to
  the `fcn` block's `u` input
- `Scope` — live view during simulation
- `out.sim_data` — logs the state trajectory to the base workspace for
  post-processing

## How to run

1. Open `model.slx` in Simulink, hit Run. This populates the `out`
   variable (with `out.sim_data`) in the base workspace.
2. Run `animate_robot.m` — it reads `out.sim_data` and renders a 2D
   stick-figure animation of the balancing motion.

Note: `animate_robot.m` will throw `Unable to resolve the name
'out.sim_data'` if run before step 1 — it depends on the Simulink
simulation having already populated that variable, it is not
standalone.

## Relationship to the main (script-based) implementation

Both implementations solve the same control problem and should produce
consistent balancing behavior; the block-diagram version demonstrates
the same LQR design in a Simulink modeling workflow rather than a pure
scripted ODE approach. The 3D visualization pipeline
(`visualize_robot_3d.m`) is currently wired to the `ode45`-based script
output only.
