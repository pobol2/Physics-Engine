# Physics Engine

[![Build and test](https://github.com/pobol2/Physics-Engine/actions/workflows/build.yml/badge.svg)](https://github.com/pobol2/Physics-Engine/actions/workflows/build.yml)

A C++17 library for classical particle mechanics. You describe particles, forces,
constraints, and moving supports; the engine integrates the equations of motion
with fixed-step or adaptive Runge–Kutta methods and reports diagnostics that make
numerical error visible. Every feature is validated against an independent
reference.

## Features

- **Forces:** gravity, harmonic wells, springs, linear and quadratic drag, central
  and pair potentials, softened `N`-body gravitation, Lorentz forces,
  time-dependent drives, and your own (`customForce` or a `physics::Force` class)
- **Constraints:** rods, links, planes, prescribed motion, and your own holonomic
  constraints, all solved together with Lagrange multipliers
- **Moving supports:** pivots, anchors, and planes can follow any `Motion`
- **Integrators:** classical RK4 and adaptive Dormand–Prince 5(4), both with
  constraint projection after each step
- **Diagnostics:** energy, external work, an energy-balance error that isolates
  numerical error even in driven systems, momentum, constraint residuals, and step
  statistics

## Build

Requires CMake 3.20+ and a C++17 compiler. Eigen 3.4.0 is downloaded
automatically.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

| Option | Default | Effect |
| --- | --- | --- |
| `PHYSICS_BUILD_VIEWER` | `OFF` | Build the OpenGL viewer (GLFW is fetched if not installed) |
| `PHYSICS_FETCH_EIGEN` | `ON` | Download Eigen 3.4.0 if no installed package is found |
| `PHYSICS_FETCH_GLFW` | `ON` | Download GLFW if no installed package is found |
| `BUILD_TESTING` | `ON` | Build the tests |

For offline builds, set `CMAKE_PREFIX_PATH` to an installed Eigen 3.4.0, or pass
`-DFETCHCONTENT_SOURCE_DIR_EIGEN=<path/to/eigen-3.4.0>`.

## Run the examples

```sh
build/physics_runner --list
build/physics_runner --example pendulum --angle 1.1 --duration 10 --csv pendulum.csv
build/physics_runner --example kepler --eccentricity 0.9 --integrator adaptive --rtol 1e-10 --atol 1e-10
build/physics_viewer --example double-pendulum --angle1 2.5 --duration 30
```

The examples are `oscillator`, `pendulum`, `driven-oscillator`,
`parametric-oscillator`, `kapitza-pendulum`, `double-pendulum`, `rotating-hoop`,
`kepler`, `three-body`, and `cyclotron`. `--list` shows each one's parameters,
units, and defaults. The runner prints a summary and, with `--csv`, writes one row
per particle per sample. Units are SI; angles are radians from the downward
vertical. With Visual Studio, executables are in `build/Release/`.

## Use the library

```cpp
#include "physics/simulation.hpp"

using namespace physics;

Model model;
model.masses = {1.0, 0.5};                             // kg
model.gravity = {0.0, -9.81, 0.0};                     // m/s^2
// Particle 0 hangs on a 1 m rod from a pivot oscillating along x.
model.distance_constraints = {{0, Motion::oscillating({0, 0, 0}, {0.2, 0, 0}, 3.0), 1.0}};
// Particle 1 hangs from particle 0 on a spring (k = 40 N/m, rest length 0.5 m).
model.springs = {{0, 1, 40.0, 0.5}};
model.forces = {quadraticDrag(1, 0.05)};

State state;
state.positions = {{0.2, -1.0, 0.0}, {0.2, -1.6, 0.0}};
state.velocities = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

Simulation sim(model, state);         // projects the state onto the constraints
sim.advanceAdaptive(10.0);            // or sim.advanceTo(10.0, 0.001) for RK4

const Diagnostics d = sim.diagnostics();
// d.energy_change - d.external_work = d.energy_balance_error: numerical error only
```

`engine/` is a self-contained CMake project, so another project can add it and link
one target. The examples, apps, and tests are not built:

```cmake
add_subdirectory(external/Physics-Engine/engine physics-engine EXCLUDE_FROM_ALL)
target_link_libraries(my_app PRIVATE physics::engine)
```

Or, with FetchContent:

```cmake
include(FetchContent)
FetchContent_Declare(physics_engine
    GIT_REPOSITORY https://github.com/pobol2/Physics-Engine.git
    GIT_TAG master            # or a specific commit, for reproducible builds
    SOURCE_SUBDIR engine)
FetchContent_MakeAvailable(physics_engine)
target_link_libraries(my_app PRIVATE physics::engine)
```

If your project already defines `Eigen3::Eigen`, the engine uses it. Your
application owns its problem setup, file output, plotting, and comparisons with
analytical results. [examples/external-project](examples/external-project/) is a
complete template.

## Layout

| Path | Contents |
| --- | --- |
| `engine/` | The library: public headers in `include/physics/`, sources in `src/` |
| `examples/` | Ten example systems, and `external-project/`, a template for using the engine |
| `apps/` | Command-line runner and optional OpenGL viewer |
| `tests/` | Unit tests, validation against references, accuracy sweeps |
| `docs/` | Theory, design, and validation results |

## Documentation

- [Physics](docs/physics.md): equations for forces, constraints, energy balance, and the examples
- [Architecture](docs/architecture.md): design, extension points, solver, and integrators
- [Validation](docs/validation.md): accuracy criteria and measured results

## Scope

Version 1 covers point particles with ideal holonomic constraints. Rigid bodies,
collisions, friction, nonholonomic constraints, and symplectic integrators are
not included. Neither integrator conserves energy exactly: check the energy-balance
error and do a step-size study for new systems. The constraint solver uses dense
blocks, so it suits mechanisms with tens of links, not large networks.

## License

[MIT](LICENSE)
