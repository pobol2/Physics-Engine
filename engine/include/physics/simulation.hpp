#pragma once

#include "physics/model.hpp"

#include <cstdint>
#include <limits>

namespace physics {
// Classical RK4; every stage is evaluated at its own unmodified state and time,
// with prescribed particles following their motion exactly. Constraints are
// projected only after combining stages, never between them.
State rk4Step(const Model& model, const State& state, double time, double dt);

// Error control for the embedded Dormand-Prince 5(4) integrator. A step is
// accepted when the RMS over free coordinates (not the work integral) of
//   error_i / (absolute_tolerance + relative_tolerance * |y_i|)
// is at most one. Positions and velocities share the tolerances, so express
// them in units where both are of order one, or tighten absolute_tolerance.
struct AdaptiveSettings {
    double relative_tolerance = 1e-9;
    double absolute_tolerance = 1e-9;
    double initial_step = 0.0; // 0 selects a step from the initial derivatives
    double max_step = std::numeric_limits<double>::infinity();
    double min_step = 1e-12;   // failure below this step size
    std::uint64_t max_steps = 10'000'000; // per advanceAdaptive call
};

struct StepStatistics {
    std::uint64_t accepted = 0;
    std::uint64_t rejected = 0;
    std::uint64_t evaluations = 0; // acceleration evaluations
    double last_step = 0.0;
    double min_step = 0.0;         // smallest and largest accepted steps
    double max_step = 0.0;
};

struct Diagnostics {
    Energy energy;
    double energy_change = 0.0;
    // Normalized by max(1 joule, abs(initial total energy)); defined at zero E.
    double scaled_energy_change = 0.0;
    // Integral of Dynamics::external_power: work done by nonconservative forces,
    // moving supports, rheonomic constraints, and time-dependent potentials.
    double external_work = 0.0;
    // energy_change - external_work. Zero for the exact solution, so this
    // measures numerical error even in driven or dissipative systems.
    double energy_balance_error = 0.0;
    double scaled_energy_balance_error = 0.0;
    Momentum momentum;
    ConstraintError constraints;
    StepStatistics steps;
};

class Simulation {
public:
    Simulation(Model model, State initial_state, double start_time = 0.0);
    // One fixed RK4 step.
    void step(double dt);
    // Fixed RK4 steps on a grid anchored at the current time; the last step is
    // shortened to land exactly on end_time.
    void advanceTo(double end_time, double max_step);
    // Error-controlled Dormand-Prince 5(4) steps, landing exactly on end_time.
    // The step-size controller's state carries over between calls, so repeated
    // calls at output times cost little extra. Rejected steps and failed stage
    // evaluations shrink the step; throws std::runtime_error below min_step.
    void advanceAdaptive(double end_time, const AdaptiveSettings& settings = {});
    const Model& model() const { return model_; }
    const State& state() const { return state_; }
    double time() const { return time_; }
    std::uint64_t steps() const { return statistics_.accepted; }
    const StepStatistics& statistics() const { return statistics_; }
    Diagnostics diagnostics() const;

private:
    void accept(State next, double work, double next_time);
    Model model_;
    State state_;
    double time_;
    double initial_energy_;
    double external_work_ = 0.0;
    StepStatistics statistics_;
    double suggested_step_ = 0.0;
    double previous_error_ = 1e-4;
};
} // namespace physics
