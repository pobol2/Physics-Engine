# Numerical validation

The engine has two integrators (see [architecture](architecture.md)):

- **Classical RK4** with constraint projection after each step. It has formal order
  four but is not symplectic, so energy is not conserved exactly.
- **Adaptive Dormand–Prince 5(4)** with error control and the same projection.

The criteria below cover the stated systems and durations. They are not guarantees
for arbitrary parameters, step sizes, or tolerances.

## Reproduce

```sh
ctest --test-dir build -C Release --output-on-failure
build/physics_numerics build/numerical-validation.csv
build/physics_dynamics_numerics build/dynamics-validation.csv
```

Both programs print every measurement, write it to the optional CSV, and fail if a
criterion is not met. CTest runs them as `numerical_accuracy` and
`dynamics_accuracy`.

## Unit tests

`physics_tests` contains focused checks of the original features against hand
calculations and closed-form solutions:

- force equals the negative numerical gradient of the potential, for every interaction
- exact trajectories: harmonic oscillator; underdamped, critical, and overdamped
  oscillators; the two-body spring with reduced mass; exponential velocity decay
  under damping
- fourth-order convergence for the oscillator and the nonlinear pendulum (checked
  against an independent angular ODE), the small-angle limit, and the
  finite-amplitude period
- 50 s energy drift and constraint drift
- constraint initialization, zero reaction power, fixed anchors, and rejection of
  singular or conflicting configurations
- input validation, failed-step rollback, exact endpoints, and CLI and CSV behavior

`physics_library_tests` covers the general-purpose features:

- every `Motion` kind: velocity and acceleration match finite differences of position
- every conservative library force (central force about a moving center, pair
  forces, softened gravitation, a custom time-dependent potential): force equals
  `-grad U`, and the reported `∂U/∂t` matches finite differences in time
- the energy identity `external_power = dE/dt`, by finite differences along the
  flow, for one system that combines every force, constraint, and support type;
  its integrated energy balance stays within `1e-7 J` (RK4) and `1e-6 J`
  (adaptive) over 2 s
- coupled constraints: projection onto two rods, mass-weighted projection
  (preserves the centre of mass and momentum), a free rigid triangle (energy and
  angular momentum conserved within `1e-8` over 10 s), and a five-link chain
- inclined-plane acceleration, and a user-defined spherical constraint that
  reproduces the built-in rod to `1e-13 m`
- adaptive control: error proportional to tolerance, exact landing on end times,
  evaluation accounting, retry after a singular trial step, and failure reporting
- diagnostics, and every catalog example under both integrators

`physics_example_tests` validates each example against an independent reference
(measured values at the listed settings):

| Example | Reference | Measured |
| --- | --- | --- |
| Driven oscillator | Exact transient + steady solution, t = 20 s | RK4 `dt = 1e-3`: `2.7e-12`; adaptive `1e-11`: `1.0e-10` |
| Parametric oscillator | Scalar Mathieu ODE; resonance growth `h ω0/4` | rate `0.02483` vs `0.025` |
| Kapitza pendulum | Angular ODE with moving pivot, t = 1 s | `1.8e-12 rad`; inverted deviation `0.24` (stable) vs `3.1` (`a ω` too small) |
| Kapitza, as particle + link | Same system with a moving-pivot rod | `< 1e-11 m` |
| Double pendulum | Lagrange's equations, t = 3 s | RK4: `6.1e-11 rad`; adaptive (682 steps): `7.3e-11 rad` |
| Rotating hoop | Reduced ODE and Jacobi integral, t = 3 s | `6.6e-11 rad`; Jacobi drift `9.0e-11`; hoop work `21.5 J` balanced to `1e-9` |
| Kepler, e = 0.6 | Closure after `2π sqrt(a³/μ)`, `E`, `L`, Runge–Lenz vector | `7.1e-11 m` in 3334 evaluations (RK4 `dt = 1e-3`: `1.2e-10 m` in 25136) |
| Figure-eight | Choreography after `T/3`, closure after `T` | `1.8e-8`, `4.1e-8` (limited by 8-digit initial data) |
| Cyclotron | Exact helix, t = 10 s | RK4: `1.3e-12 m`; adaptive: `4.7e-11 m`; work `< 1e-13 J` |

## Accuracy sweep: oscillator and pendulum

The sweep integrates each system for 50 s with time steps `dt = 0.04, 0.02, 0.01,
0.005 s`.

- **Oscillator:** `m = 2 kg`, `k = 8 N/m`, `x0 = 0.7 m`, `v0 = -0.3 m/s`
  (`ω = 2 rad/s`), compared with the exact solution.
- **Pendulum:** `m = 2 kg`, `l = 1.3 m`, `g = 9.81 m/s²`, released from rest at
  `θ0 = 0.1, 1.1, 2.5 rad`. The trajectory is compared with a separately
  integrated angular ODE, converged to `< 1e-10`. The period is compared with
  `4 sqrt(l/g) K(sin(θ0/2))`, computed by the arithmetic–geometric mean and
  cross-checked with Simpson quadrature to `2e-13 s`. Measured periods average
  five cycles, with zero crossings located by cubic Hermite interpolation.

Phase-space error is `sqrt(‖Δq‖² + ‖Δv‖²/ω²)`, the maximum over the first 10 s.

### Acceptance criteria

| Metric (at `dt = 0.005 s`) | Oscillator | Pendulum |
| --- | ---: | ---: |
| Phase-space error | `2e-9 m` | `5e-7 rad` |
| Max 50 s energy change | `1e-8 J` | `2e-6 J` |
| Energy change / excitation energy | `5e-8` | `5e-8` |
| Period error | – | `2e-7 s` |

At every step size, the rod-length error and radial velocity must stay at or below
`1e-13`. Convergence rates `log2(error(h)/error(h/2))` must be at least 3.7
(oscillator) or 3.4 (pendulum) for trajectories and at least 4.5 for energy.
Period errors must decrease at every refinement and reach a rate of at least 3.0
at the finest step.

### Measured results

GCC 13.2 (MinGW-w64), Release. MSVC builds give the same pass/fail results.

| Oscillator dt (s) | Phase error (m) | 50 s energy change (J) | Rate |
| ---: | ---: | ---: | ---: |
| 0.04 | 4.887e-6 | 9.322e-6 | – |
| 0.02 | 3.054e-7 | 2.915e-7 | 4.00 |
| 0.01 | 1.909e-8 | 9.111e-9 | 4.00 |
| 0.005 | 1.193e-9 | 2.847e-10 | 4.00 |

| Pendulum θ0 (rad) | dt (s) | Phase error (rad) | Period error (s) | 50 s energy change (J) |
| ---: | ---: | ---: | ---: | ---: |
| 0.1 | 0.04 | 3.343e-6 | 2.773e-6 | 3.946e-6 |
| 0.1 | 0.005 | 8.173e-10 | 6.811e-10 | 1.206e-10 |
| 1.1 | 0.04 | 3.628e-5 | 1.118e-6 | 2.224e-3 |
| 1.1 | 0.005 | 1.555e-8 | 1.337e-9 | 7.171e-8 |
| 2.5 | 0.04 | 6.136e-3 | 1.023e-3 | 3.848e-2 |
| 2.5 | 0.005 | 1.147e-7 | 2.511e-8 | 1.277e-6 |

The reference periods are 2.288698, 2.473279, and 3.757941 s. Across all pendulum
runs, the rod-length error stayed below `1.4e-15 m` and the radial velocity below
`1.4e-15 m/s`. Even with exact geometry, a coarse step at large amplitude still
gives a visible phase and energy error. Projection does not make a large step
accurate. The default `dt = 0.001 s` is well inside the validated range for these
systems. Other frequencies, amplitudes near the separatrix, or strong damping need
their own step-size study.

## Dynamics sweep: coupled, moving, and rheonomic systems

`physics_dynamics_numerics` measures fixed-step RK4 convergence on systems that
exercise each V1 feature. Each reference is integrated separately in generalized
coordinates and checked against itself at half the step (difference `< 2e-12`).

- **Double pendulum** (coupled rod and link): `m = 1, 1.5 kg`, `l = 1, 0.8 m`,
  `θ = 1.2, 0.6 rad`, `θ2' = 0.5 rad/s`, t = 3 s. Error in angles and scaled rates.
- **Kapitza pendulum** (moving support): `a = 0.05 m`, `ω = 200 rad/s`,
  `θ0 = 2.9 rad`, t = 1 s.
- **Rotating hoop** (rheonomic plane): `R = 1 m`, `Ω = 5 rad/s`, `θ0 = 0.3 rad`,
  t = 3 s.
- **Parametric oscillator** (custom time-dependent potential): `ω0 = 1 rad/s`,
  `h = 0.1`, `γ = 2 rad/s`, t = 40 s. Error relative to the amplitude.

| System | dt (s) | Error | Order | `|ΔE - W|` (J) |
| --- | ---: | ---: | ---: | ---: |
| Double pendulum | 0.008 | 1.307e-6 | – | 2.4e-6 |
| | 0.004 | 1.022e-7 | 3.68 | 6.9e-8 |
| | 0.002 | 7.038e-9 | 3.86 | 1.8e-9 |
| | 0.001 | 4.603e-10 | 3.94 | 3.7e-11 |
| Kapitza pendulum | 4e-4 | 9.002e-10 | – | 2.1e-5 |
| | 2e-4 | 4.286e-11 | 4.39 | 6.3e-7 |
| | 1e-4 | 2.235e-12 | 4.26 | 1.8e-8 |
| | 5e-5 | 1.030e-13 | 4.44 | 4.5e-10 |
| Rotating hoop | 0.01 | 4.601e-7 | – | 4.5e-7 |
| | 0.005 | 3.617e-8 | 3.67 | 1.5e-8 |
| | 0.0025 | 2.492e-9 | 3.86 | 2.3e-9 |
| | 0.00125 | 1.631e-10 | 3.93 | 1.9e-10 |
| Parametric oscillator | 0.04 | 6.297e-7 | – | 9.1e-12 |
| | 0.02 | 3.873e-8 | 4.02 | 3.9e-13 |
| | 0.01 | 2.401e-9 | 4.01 | 1.8e-14 |
| | 0.005 | 1.494e-10 | 4.01 | 9.7e-16 |

The constraint residual stayed below `4e-16` in every run. The hoop is
pre-asymptotic at `Ω dt = 0.1` (order 2.96 from `dt = 0.02` to `0.01`, not part of
the criteria). The balance error converges at orders between 3.6 and 6, usually
faster than the trajectory error, because the work integral is integrated with the
state. In the Kapitza pendulum the pivot does about 36 J of work in the first second
through rapidly oscillating power, so its balance error is the largest.

### Adaptive integration

Kepler orbits from periapsis over one period (`μ = a = m = 1`), with the closure
distance as the error. Tolerances are `rtol = atol`. Order is
`log10(error ratio)/2` per hundredfold tolerance reduction, so 1 means
proportional.

| Orbit | Method | dt or tol | Closure error (m) | Evaluations | Order |
| --- | --- | ---: | ---: | ---: | ---: |
| e = 0.6 | RK4 | 0.002 | 1.995e-9 | 12568 | 4.05 |
| | RK4 | 0.001 | 1.227e-10 | 25136 | 4.02 |
| | RK4 | 0.0005 | 7.529e-12 | 50268 | 4.03 |
| | adaptive | 1e-8 | 1.537e-6 | 548 | 1.02 |
| | adaptive | 1e-10 | 1.040e-8 | 1332 | 1.09 |
| | adaptive | 1e-12 | 7.080e-11 | 3334 | 1.08 |
| e = 0.9 | RK4 | 0.002 | 1.283e-4 | 12568 | 4.26 |
| | RK4 | 0.001 | 7.257e-6 | 25136 | 4.14 |
| | RK4 | 0.0005 | 4.300e-7 | 50268 | 4.08 |
| | adaptive | 1e-8 | 8.202e-6 | 884 | 1.02 |
| | adaptive | 1e-10 | 5.415e-8 | 1983 | 1.09 |
| | adaptive | 1e-12 | 3.627e-10 | 4972 | 1.09 |

The global error is roughly 70–900 times the tolerance after one orbit and scales
with it. For e = 0.6, the steps range from `2.2e-4` to `3.0e-2 s`. The advantage
grows with eccentricity: at e = 0.9, tolerance `1e-12` is about 1200 times more
accurate than RK4 at `dt = 5e-4`, for a tenth of the evaluations.

The figure-eight orbit closes to `3.1e-7`, `4.7e-8`, and `4.1e-8` at tolerances
`1e-8`, `1e-10`, and `1e-12`. The floor comes from the 8-digit initial conditions
and period, not from the integrator. The energy change at `1e-12` is `1.7e-11`.

### Acceptance criteria

- RK4 order of at least 3.6 at every refinement shown, and a constraint residual of
  at most `1e-13`.
- Finest-step error of at most `1e-9` for the double pendulum, Kapitza pendulum, and
  rotating hoop.
- For both orbits: adaptive error falls at every tolerance step and stays at or
  below `1e4 × tol`. At tolerance `1e-12`, adaptive is more accurate than RK4 at
  `dt = 1e-3` with at most a fifth of its evaluations.
- Figure-eight closure of at most `1e-6` at tolerance `1e-12`.

### Choosing an integrator

Use RK4 when a fixed step is natural (fixed output grids, smooth motion on one time
scale) and validate it with a step-size study. Use the adaptive integrator when time
scales vary along the trajectory (eccentric orbits, close encounters, sharp
bounces) or when a tolerance is easier to choose than a step. Positions and
velocities share the tolerance, so nondimensionalize, or set `atol` for the smaller
of the two scales. Under adaptive stepping, the energy-balance error also contains
the work integral's quadrature error. That error grows with the number of steps and
is largest for strong, rapidly oscillating drives. Tighten the tolerance for a
sharper check.
