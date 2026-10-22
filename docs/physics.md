# Physics

All quantities are in SI units. The engine integrates the Cartesian positions `q`
and velocities `v` of point particles with masses `m > 0`, subject to applied forces
and ideal holonomic constraints that may depend on time.

## Forces and potentials

Every force is either **conservative**, deriving from a potential `U(q, t)` that is
part of the energy, or **nonconservative**, in which case its power counts as
external work (see [Energy balance](#energy-balance)).

### Built-in model fields

| Interaction | Force | Potential energy |
| --- | --- | --- |
| Uniform gravity `g` | `F = m g` | `U = -m g · q` |
| Harmonic well about `q0(t)` | `F = -k (q - q0(t))` | `U = (k/2) ‖q - q0(t)‖²` |
| Spring between particles 1 and 2 | `F1 = -k (r - l0) d/r`, `F2 = -F1` | `U = (k/2) (r - l0)²` |
| Linear damping | `F = -c v` | none |

For springs, `d = q1 - q2`, `r = ‖d‖`, and `l0 ≥ 0` is the rest length. Springs
work in both tension and compression. For zero rest length the force is `-k d`,
which stays defined at `r = 0`. With a positive rest length, coincident endpoints
are rejected because the force direction is undefined. The well's equilibrium is a
[motion](#prescribed-motion), so it may move. Moving it makes the potential depend
explicitly on time, with `∂U/∂t = -k (q - q0) · q0'`.

### Force library (`force.hpp`)

| Factory | Force | Kind |
| --- | --- | --- |
| `drivingForce(i, F(t))`, `harmonicDrive(i, A, Ω, φ)` | `F(t)`; `A cos(Ωt + φ)` | external |
| `centralForce(i, center(t), U)` | `-U'(r) r̂`, `r = q - center(t)` | conservative |
| `pairForce(i, j, U)` | `-U'(r) r̂` on `i`, opposite on `j` | conservative |
| `gravitation(G, particles, ε)` | Newtonian `N`-body, softened by `ε` | conservative |
| `quadraticDrag(i, c)` | `-c ‖v‖ v` | dissipative |
| `lorentzForce(i, q, E, B)` | `q (E(q, t) + v × B(q, t))` | external |
| `customForce({...})` | user functions | either |

Radial potentials come from `inverseSquare(k)` (`U = -k/r`) and
`lennardJones(ε, σ)`, or from any pair of functions `U(r)`, `U'(r)`. Gravitation
uses `U = -G mᵢ mⱼ / sqrt(r² + ε²)`. A magnetic force does no work, so its power is
zero up to rounding. A custom force supplies a function that adds its forces. If it
also supplies `U(q, t)`, it is conservative, and a time-dependent `U` must supply
`∂U/∂t` as well. Every library force is checked against its potential by finite
differences in the tests.

## Constraints

Constraints are ideal: their reactions do no work under virtual displacements. Any
number of constraints may act on one particle. They are solved together.

### Prescribed motion

A `Motion` gives a support's position, velocity, and acceleration as functions of
time:

| Motion | Position |
| --- | --- |
| `stationary(p)` | `p` |
| `linear(p0, u)` | `p0 + u t` |
| `oscillating(c, A, ω, φ)` | `c + A cos(ωt + φ)` |
| `circular(c, r0, Ω)` | `c + R(Ω t) r0`, rotation about `Ω` by `‖Ω‖ t` |
| `custom(p, p', p'')` | user functions with consistent derivatives |

A **prescribed particle** follows its motion exactly. Its acceleration is `p''(t)`,
and constraint reactions do not move it, as if its mass were infinite. Its own mass
still counts in energy and momentum. Pivots, plane points, and harmonic equilibria
also accept a `Motion`. A moving support can therefore be written as a moving pivot
or as a prescribed particle joined by a link; the tests check that both give the
same motion.

### Holonomic constraints

| Constraint | Equation `C(q, t) = 0` |
| --- | --- |
| Distance from a pivot (rod) | `‖q - p(t)‖ - l` |
| Link between particles (rod) | `‖q1 - q2‖ - l` |
| Plane (`planeConstraint`) | `n(t) · (q - p(t))`, `n(t)` rotating at constant `Ω` |
| Custom (`customConstraint`, `Constraint`) | user rows |

Each scalar constraint provides a row: the value `C`, the gradients `J = ∂C/∂q`,
the explicit rate `∂C/∂t`, and an acceleration bias `b`. The bias is chosen so that
`J a + b = 0` is the second time derivative of `C = 0` on the constraint manifold.
For a rod with `d = q - p` and `u = v - p'`:

```text
J = d/‖d‖,   ∂C/∂t = -J · p',   b = ‖u‖²/‖d‖ - J · p''
```

For the plane, with `e = q - p`, `n' = Ω × n`, and `n'' = Ω × n'`:

```text
J = n,   ∂C/∂t = n' · e - n · p',   b = -n · p'' + 2 n' · (v - p') + n'' · e
```

### Lagrange multipliers

With `W` the diagonal of inverse masses (zero for prescribed particles) and
`a_free = F/m`, the multipliers solve

```text
(J W Jᵀ) λ = -(J a_free + b),        a = a_free + W Jᵀ λ.
```

Rows that share no free particle are independent, so each coupled block is solved
on its own. A single row is solved exactly. A larger block uses a complete
orthogonal decomposition, which gives the minimum-norm solution when constraints
are redundant but consistent. For one rod from a fixed pivot this reduces to

```text
a = a_free - d (d · a_free + v · v) / (d · d).
```

### Projection

Integration enforces constraints only at acceleration level, so positions and
velocities drift slowly. After every accepted step, the state is projected back
onto the constraints:

1. Prescribed particles are set to their motion.
2. Positions take Newton steps `Δq = -W Jᵀ (J W Jᵀ)⁻¹ C`. This is the
   mass-weighted smallest correction, so it preserves the centre of mass and
   momentum of a free system. Iteration continues to rounding level. One step is
   exact for an isolated rod.
3. Velocities are corrected the same way so that `J v + ∂C/∂t = 0`.

Inconsistent constraints (for example, a sphere and a disjoint plane) and singular
geometry (a particle at its own pivot) are rejected without changing the state.

## Energy balance

The energy is `E = T + U`, with `T = Σ (m/2) ‖v‖²` and `U` the sum of all
potentials, both time-dependent. Along the equations of motion,

```text
dE/dt = P_ext = Σ (m a - F_conservative) · v + ∂U/∂t
```

This collects the power of nonconservative forces, of constraint reactions (nonzero
for moving or rotating constraints), of the drives that move prescribed particles,
and of explicit time dependence. The simulation integrates `W = ∫ P_ext dt`
alongside the state with the same integrator, and reports:

- `energy_change = E(t) - E(0)`, the physical change plus numerical error
- `external_work = W(t)`
- `energy_balance_error = energy_change - external_work`, which is zero for the
  exact solution even in driven, damped, or rheonomic systems

`W` is a diagnostic. It does not influence adaptive step sizes. Its quadrature error
adds up over steps, so with strong, rapidly oscillating power (the Kapitza pendulum)
the balance error is the least accurate diagnostic.

The simulation also reports total linear momentum `Σ m v` and angular momentum
about the origin `Σ q × m v`, including prescribed particles.

## Examples

Angles are measured from the downward vertical (`-y`), positive toward `+x`.

**Harmonic and driven oscillators.** One particle in a harmonic well along `x`:

```text
m x'' + c x' + k x = F0 cos(Ω t),   ω0 = sqrt(k/m)
steady amplitude F0 / sqrt((k - m Ω²)² + (c Ω)²),  phase lag atan2(c Ω, k - m Ω²)
```

**Parametric oscillator** (custom time-dependent potential):

```text
x'' + ω0² (1 + h cos(γ t)) x = 0,   U = (m ω0²/2)(1 + h cos(γ t)) x²
```

Near `γ = 2 ω0`, the amplitude grows as `exp(s t)` with `s = h ω0 / 4` to first
order in `h`.

**Simple pendulum.** A rod of length `l` from a fixed pivot:

```text
θ'' + (c/m) θ' + (g/l) sin θ = 0
period = 4 sqrt(l/g) K(sin(θmax/2))      (c = 0, released from rest at θmax)
```

where `K` is the complete elliptic integral of the first kind.

**Kapitza pendulum.** The pivot moves as `y_p = a cos(ω t)`:

```text
θ'' = -(g + y_p'') sin θ / l = -(g - a ω² cos ω t) sin θ / l
```

The inverted position is stable when `(a ω)² > 2 g l`.

**Double pendulum.** A rod from the pivot to mass 1 and a link from mass 1 to
mass 2. The reference is Lagrange's equations in `θ1`, `θ2`.

**Folded chain.** `N` equal masses joined by rigid links, the first on a rod from
the pivot, released at rest in a narrow U with the free end beside the pivot.
Optional springs between masses `i - 1` and `i + 1`, relaxed when the chain is
straight, resist sharp kinks. In the energy-conserving continuum model (Calkin and
March, 1989), after the free end of a chain of length `L` falls `x`:

```text
v² = g x (2L - x) / (L - x)      (free fall: v² = 2 g x)
```

The free end starts accelerating at `g` and then outruns free fall. The discrete chain converges
to this model as the links get shorter and the fold narrower.

**Bead on a rotating hoop.** Radius `R`, spinning at `Ω` about the vertical,
modelled as a distance constraint plus a rotating plane:

```text
θ'' = sin θ (Ω² cos θ - g/R)
h = (R²/2) θ'² - (R²/2) Ω² sin²θ - g R cos θ      (Jacobi integral, per unit mass)
```

For `Ω² > g/R`, the bead rests at `cos θ0 = g/(R Ω²)`. The hoop's motor does work,
so `E` changes while `h` is conserved.

**Kepler orbit.** Inverse-square attraction `U = -μ m/r` toward a fixed center,
started at periapsis `a (1 - e)`:

```text
period = 2π sqrt(a³/μ),   E = -μ m / 2a,   A = v × (q × v) - μ q/r = μ e x̂
```

**Figure-eight three-body orbit.** Three equal masses under mutual gravity on the
Chenciner–Montgomery choreography: each body follows the next one's path a third of
a period later. The period is 6.32591398 for `G = m = 1` and scales as
`1/sqrt(G m)`.

**Cyclotron.** A charge in a uniform field `B ẑ` moves on a helix with
`ω = q B/m` and radius `m v⊥/(q B)`.

## Connection to Landau and Lifshitz

Landau and Lifshitz's *Mechanics* (Vol. 1 of the Course of Theoretical Physics) is
one application of the engine. The tests reproduce these results:

| Result | Section | Test |
| --- | --- | --- |
| Lagrangian of a double pendulum | §5, Problem 1 | `example_double_pendulum`, `dynamics_accuracy` |
| Pendulum with a moving point of support | §5, Problem 3 | `example_kapitza_pendulum` |
| Period of a pendulum as a function of amplitude | §11, Problem 1 | `pendulum_period`, `numerical_accuracy` |
| Reduction of the two-body problem to reduced mass | §13 | `spring_rest_length` |
| Kepler's problem and the Laplace–Runge–Lenz vector | §15 | `example_kepler` |
| Free one-dimensional oscillations | §21 | `oscillator_accuracy`, `oscillator_convergence` |
| Forced oscillations | §22 | `example_driven_oscillator` |
| Damped oscillations: underdamped, critical, overdamped | §25 | `damped_examples` |
| Forced oscillations under friction | §26 | `example_driven_oscillator` |
| Parametric resonance | §27 | `example_parametric_oscillator` |
| Motion in a rapidly oscillating field (Kapitza) | §30 | `example_kapitza_pendulum` |
