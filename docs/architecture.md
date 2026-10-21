# Architecture

## Layers

| Target | Directory | Role | Depends on |
| --- | --- | --- | --- |
| `physics_engine` (`physics::engine`) | `engine/` | The library: state, models, forces, constraints, integrators, diagnostics | Eigen |
| `physics_examples` | `examples/` | Example systems and a name-based catalog | engine |
| `physics_app` | `apps/` | Option parsing, run loop, CSV output | engine, examples |
| `physics_runner`, `physics_viewer` | `apps/` | Command-line and OpenGL front ends | app (+ GLFW) |

The engine has no I/O, rendering, clocks, or global state. `engine/` has its own
`CMakeLists.txt`, so it builds alone or inside another project. Everything else
uses only the public API, as an external application would; file output,
visualization, and problem-specific analysis stay outside the library. Vectors are
`Eigen::Vector3d`.

| Header (`engine/include/physics/`) | Contents |
| --- | --- |
| `state.hpp` | `State`, `SystemView` |
| `motion.hpp` | `Motion` (prescribed trajectories) |
| `force.hpp` | `Force` interface and force library |
| `constraint.hpp` | `ConstraintRow`, `Constraint` interface, plane and custom constraints |
| `model.hpp` | `Model`, built-in interactions, evaluation functions |
| `simulation.hpp` | `rk4Step`, `AdaptiveSettings`, `Simulation`, `Diagnostics` |

## Data and ownership

- `Model` holds configuration: masses, gravity, built-in forces and constraints,
  and shared pointers to `Force` and `Constraint` objects. `State` holds only
  positions and velocities. Both are values. Copying a model shares its force and
  constraint objects, which are immutable and must stay free of mutable state.
  `Motion` is likewise an immutable value.
- `Simulation` keeps its own validated copy of both. It exposes them as `const`
  and owns the time, step statistics, the external-work integral, the energy
  baseline, and the adaptive step-size controller's state.
- Evaluation functions (`evaluateForces`, `evaluateDynamics`, `energy`, `momentum`,
  `constraintError`) are pure. They validate their inputs, compute fresh results
  from the supplied state and time, and never accumulate or mutate.
- Invalid input (non-positive masses, out-of-range indices, non-finite values,
  singular or inconsistent constraints) throws `std::invalid_argument`. A rejected
  fixed step leaves the state, time, and statistics unchanged.

## Extension points

**Forces.** Implement `physics::Force`, or wrap functions with `customForce`.
`accumulate` adds forces for a `SystemView` (masses, state, time). A conservative
force also implements `potential` and, if its potential depends on time,
`potentialRate`. Otherwise its power counts as external work.

**Constraints.** Implement `physics::Constraint`, or wrap a function with
`customConstraint`. `appendRows` emits one `ConstraintRow` per scalar equation: the
value, the gradient for each particle involved, `∂C/∂t`, and an acceleration bias
(see [physics](physics.md#holonomic-constraints)). The solver does the rest:
coupling with other constraints, multipliers, projection, and diagnostics.

**Supports.** Anything that takes a `Motion` can move: pivots, plane points,
harmonic equilibria, and prescribed particles. Use `Motion::custom` for arbitrary
trajectories, supplying consistent velocity and acceleration.

## Constraint solving

At each evaluation, the built-in constraints and the `Constraint` objects produce
rows. Rows are grouped into blocks connected through free (non-prescribed)
particles, using union–find. Each block's `J W Jᵀ` system is solved exactly for a
single row, or by complete orthogonal decomposition for several rows, which gives
the minimum-norm solution for redundant constraints. The same machinery serves
three levels:

| Level | Right-hand side | Used by |
| --- | --- | --- |
| Acceleration | `-(J a_free + b)` | every derivative evaluation |
| Position | `C` (Newton iteration) | projection |
| Velocity | `J v + ∂C/∂t` | projection |

Dense blocks cost `O(rows³)`. That suits mechanisms and chains of tens of links,
not ropes or cloth with thousands.

## Integration

Both integrators operate on `y = (q, v, W)`. Here `W` is the external-work integral,
whose derivative is `Dynamics::external_power`. At every stage, prescribed particles
are placed on their motion before forces are evaluated. Stages are evaluated at
their own unmodified state and time. Constraints are projected only after the stages
are combined, not between stages. The same projection runs when a `Simulation` is
constructed, so an off-constraint initial state is repaired before the energy
baseline is measured.

- `step(dt)` and `advanceTo(end, max_step)` take fixed classical RK4 steps. The grid
  is anchored at the start time, and the last step is shortened to land exactly on
  `end`.
- `advanceAdaptive(end, settings)` uses Dormand–Prince 5(4), propagating the
  fifth-order solution (seven evaluations per attempt). The embedded error estimate
  is measured in an RMS norm over free coordinates, with
  `atol + rtol · |y|` weights. Prescribed particles and `W` are excluded.
  - The controller is the PI rule from Hairer, Nørsett and Wanner (exponents 0.17
    and 0.04, safety 0.9, factor range [0.2, 10], no growth right after a
    rejection). The first step comes from the size of the initial derivatives.
  - A step whose stages hit a singular configuration, or whose projection fails,
    counts as rejected and is retried four times smaller.
  - The controller's state persists across calls. Advancing to a sequence of
    output times therefore costs at most one extra step per output, and a
    shortened final step does not reduce the next step.
  - The integrator throws `std::runtime_error` if the step falls below `min_step`
    or more than `max_steps` attempts are needed. Steps accepted before the failure
    are kept.

The viewer uses wall-clock time only to pace playback. It never changes the
physics step.

## Diagnostics

`Simulation::diagnostics()` reports:

- energy and its change since the start
- external work and the energy-balance error, `ΔE - W`
- linear and angular momentum
- the largest position and velocity constraint residuals, including prescribed
  particles
- step statistics: accepted and rejected steps, acceleration evaluations, and the
  smallest, largest, and last step

The energy and balance errors are also given relative to `max(1 J, |E0|)`.

## Extending

- **New force:** prefer a `Force` implementation over a new `Model` field. Add a
  test that the force equals the negative numerical gradient of its potential, and
  that `external_power` equals the numerical `dE/dt` (see `force_library` and
  `external_power` in `tests/library_tests.cpp`).
- **New constraint:** implement `Constraint`. Test the rows against finite
  differences of `C`, and check a trajectory against an independent
  generalized-coordinate reference.
- **New integrator:** add it to `engine/src/simulation.cpp`, integrate `W`
  alongside the state, and respect the projection policy above.
- **New example:** add a typed config and factory to `examples/examples.hpp`,
  register it in `examples/catalog.cpp`, and validate it against an independent reference
  in `tests/example_tests.cpp`.
