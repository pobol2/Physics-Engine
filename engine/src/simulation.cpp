#include "physics/simulation.hpp"

#include "unchecked.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

namespace physics {
namespace {
// Integrated variables: positions, velocities, then the external-work integral.
using Vector = Eigen::VectorXd;

void positiveStep(double dt) {
    if (!std::isfinite(dt) || dt <= 0.0) {
        throw std::invalid_argument("Time step must be finite and positive");
    }
}

void advancingStep(double time, double dt) {
    positiveStep(dt);
    if (!std::isfinite(time) || !std::isfinite(time + dt) || time + dt <= time) {
        throw std::invalid_argument("Time step must advance a finite simulation time");
    }
}

Vector pack(const State& state, double work) {
    const auto n = static_cast<Eigen::Index>(state.positions.size());
    Vector y(6 * n + 1);
    for (Eigen::Index i = 0; i < n; ++i) {
        y.segment<3>(3 * i) = state.positions[i];
        y.segment<3>(3 * (n + i)) = state.velocities[i];
    }
    y[6 * n] = work;
    return y;
}

State unpack(const Vector& y) {
    const Eigen::Index n = (y.size() - 1) / 6;
    State state{Vectors(static_cast<std::size_t>(n)), Vectors(static_cast<std::size_t>(n))};
    for (Eigen::Index i = 0; i < n; ++i) {
        state.positions[i] = y.segment<3>(3 * i);
        state.velocities[i] = y.segment<3>(3 * (n + i));
    }
    return state;
}

class System {
public:
    System(const Model& model, StepStatistics& statistics) : model_(model), statistics_(statistics) {
        free_.assign(model.masses.size(), true);
        for (const auto& prescribed : model.prescribed) free_[prescribed.particle] = false;
    }

    // Prescribed particles follow their motion exactly at every stage. The model
    // was validated before integration; only the stage state can go bad.
    Vector derivative(double time, const Vector& y) const {
        State stage = unpack(y);
        applyPrescribedMotion(model_, stage, time);
        validateState(model_, stage);
        const auto dynamics = unchecked::evaluateDynamics(model_, stage, time);
        ++statistics_.evaluations;
        const auto n = static_cast<Eigen::Index>(stage.positions.size());
        Vector dy(y.size());
        for (Eigen::Index i = 0; i < n; ++i) {
            dy.segment<3>(3 * i) = stage.velocities[i];
            dy.segment<3>(3 * (n + i)) = dynamics.accelerations[i];
        }
        dy[6 * n] = dynamics.external_power;
        return dy;
    }

    // RMS of error_i / (atol + rtol * max(|a_i|, |b_i|)) over free coordinates.
    // Prescribed particles follow their motion exactly. The work integral is a
    // diagnostic and deliberately does not influence the step size.
    double errorNorm(const Vector& error, const Vector& a, const Vector& b, const AdaptiveSettings& settings) const {
        const auto n = static_cast<Eigen::Index>(free_.size());
        double sum = 0.0;
        Eigen::Index count = 0;
        const auto add = [&](Eigen::Index k) {
            const double scale = settings.absolute_tolerance
                + settings.relative_tolerance * std::max(std::abs(a[k]), std::abs(b[k]));
            sum += (error[k] / scale) * (error[k] / scale);
            ++count;
        };
        for (Eigen::Index i = 0; i < n; ++i) {
            if (!free_[i]) continue;
            for (Eigen::Index k = 0; k < 3; ++k) {
                add(3 * i + k);
                add(3 * (n + i) + k);
            }
        }
        return count == 0 ? 0.0 : std::sqrt(sum / static_cast<double>(count));
    }

    Vector rk4(double t, const Vector& y, double h) const {
        const Vector k1 = derivative(t, y);
        const Vector k2 = derivative(t + h / 2.0, y + k1 * (h / 2.0));
        const Vector k3 = derivative(t + h / 2.0, y + k2 * (h / 2.0));
        const Vector k4 = derivative(t + h, y + k3 * h);
        return y + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0);
    }

    // Dormand-Prince 5(4): returns the fifth-order solution and the difference
    // from the embedded fourth-order solution (Hairer, Norsett & Wanner, II.5).
    std::pair<Vector, Vector> dormandPrince(double t, const Vector& y, double h) const {
        const Vector k1 = derivative(t, y);
        const Vector k2 = derivative(t + h / 5.0, y + h * (k1 / 5.0));
        const Vector k3 = derivative(t + 3.0 * h / 10.0, y + h * (3.0 / 40.0 * k1 + 9.0 / 40.0 * k2));
        const Vector k4 = derivative(t + 4.0 * h / 5.0,
            y + h * (44.0 / 45.0 * k1 - 56.0 / 15.0 * k2 + 32.0 / 9.0 * k3));
        const Vector k5 = derivative(t + 8.0 * h / 9.0,
            y + h * (19372.0 / 6561.0 * k1 - 25360.0 / 2187.0 * k2 + 64448.0 / 6561.0 * k3 - 212.0 / 729.0 * k4));
        const Vector k6 = derivative(t + h,
            y + h * (9017.0 / 3168.0 * k1 - 355.0 / 33.0 * k2 + 46732.0 / 5247.0 * k3 + 49.0 / 176.0 * k4
                     - 5103.0 / 18656.0 * k5));
        const Vector next = y + h * (35.0 / 384.0 * k1 + 500.0 / 1113.0 * k3 + 125.0 / 192.0 * k4
                                     - 2187.0 / 6784.0 * k5 + 11.0 / 84.0 * k6);
        const Vector k7 = derivative(t + h, next);
        const Vector error = h * (71.0 / 57600.0 * k1 - 71.0 / 16695.0 * k3 + 71.0 / 1920.0 * k4
                                  - 17253.0 / 339200.0 * k5 + 22.0 / 525.0 * k6 - 1.0 / 40.0 * k7);
        return {next, error};
    }

    // Starting step from the size of the derivatives (Hairer, Norsett & Wanner, II.4).
    double initialStep(double t, const Vector& y, const AdaptiveSettings& settings) const {
        const Vector f0 = derivative(t, y);
        const Vector zero = Vector::Zero(y.size());
        const double d0 = errorNorm(y, y, zero, settings);
        const double d1 = errorNorm(f0, y, zero, settings);
        double h0 = (d0 < 1e-5 || d1 < 1e-5) ? 1e-6 : 0.01 * d0 / d1;
        h0 = std::min(h0, settings.max_step);
        const Vector f1 = derivative(t + h0, y + h0 * f0);
        const double d2 = errorNorm(f1 - f0, y, zero, settings) / h0;
        const double largest = std::max(d1, d2);
        const double h1 = largest <= 1e-15 ? std::max(1e-6, h0 * 1e-3) : std::pow(0.01 / largest, 0.2);
        return std::min({100.0 * h0, h1, settings.max_step});
    }

private:
    const Model& model_;
    StepStatistics& statistics_;
    std::vector<bool> free_;
};

void validateSettings(const AdaptiveSettings& settings) {
    const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
    if (!positive(settings.relative_tolerance) || !std::isfinite(settings.absolute_tolerance)
        || settings.absolute_tolerance < 0.0
        || (settings.relative_tolerance == 0.0 && settings.absolute_tolerance == 0.0)) {
        throw std::invalid_argument("Tolerances must be finite, nonnegative, and not both zero");
    }
    if (!(settings.max_step > 0.0) || !positive(settings.min_step) || settings.min_step > settings.max_step
        || !std::isfinite(settings.initial_step) || settings.initial_step < 0.0 || settings.max_steps == 0) {
        throw std::invalid_argument("Adaptive step limits must be positive with min_step <= max_step");
    }
}
} // namespace

State rk4Step(const Model& model, const State& state, double time, double dt) {
    advancingStep(time, dt);
    validateModel(model);
    validateState(model, state);
    StepStatistics unused;
    State result = unpack(System(model, unused).rk4(time, pack(state, 0.0), dt));
    // Projection validates the combined state before accepting it.
    unchecked::projectConstraints(model, result, time + dt);
    return result;
}

Simulation::Simulation(Model model, State initial_state, double start_time)
    : model_(std::move(model)), state_(std::move(initial_state)), time_(start_time) {
    if (!std::isfinite(start_time)) throw std::invalid_argument("Start time must be finite");
    projectConstraints(model_, state_, time_); // Validates model and state.
    initial_energy_ = energy(model_, state_, time_).total();
    if (!std::isfinite(initial_energy_)) throw std::invalid_argument("Initial energy must be finite");
}

void Simulation::accept(State next, double work, double next_time) {
    const double dt = next_time - time_;
    if (!std::isfinite(unchecked::energy(model_, next, next_time).total()) || !std::isfinite(work)) {
        throw std::runtime_error("Non-finite energy after step");
    }
    if (statistics_.accepted == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error("Step counter overflow");
    }
    state_ = std::move(next);
    time_ = next_time;
    external_work_ = work;
    statistics_.min_step = statistics_.accepted == 0 ? dt : std::min(statistics_.min_step, dt);
    statistics_.max_step = std::max(statistics_.max_step, dt);
    statistics_.last_step = dt;
    ++statistics_.accepted;
}

void Simulation::step(double dt) {
    advancingStep(time_, dt);
    StepStatistics trial = statistics_;
    const Vector y = System(model_, trial).rk4(time_, pack(state_, external_work_), dt);
    State next = unpack(y);
    unchecked::projectConstraints(model_, next, time_ + dt);
    const double work = y[y.size() - 1];
    statistics_.evaluations = trial.evaluations;
    accept(std::move(next), work, time_ + dt);
}

void Simulation::advanceTo(double end_time, double max_step) {
    positiveStep(max_step);
    if (!std::isfinite(end_time) || end_time < time_) throw std::invalid_argument("End time cannot go backwards");
    const double start_time = time_;
    std::uint64_t index = 0;
    while (time_ < end_time) {
        if (index == std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("Step counter overflow");
        const double next_time = std::min(end_time, start_time + max_step * static_cast<double>(++index));
        step(next_time - time_);
    }
}

void Simulation::advanceAdaptive(double end_time, const AdaptiveSettings& settings) {
    validateSettings(settings);
    if (!std::isfinite(end_time) || end_time < time_) throw std::invalid_argument("End time cannot go backwards");
    if (end_time == time_) return;
    const System system(model_, statistics_);
    double h = suggested_step_;
    if (h <= 0.0) {
        h = settings.initial_step > 0.0 ? settings.initial_step
                                        : system.initialStep(time_, pack(state_, external_work_), settings);
    }
    h = std::min(std::max(h, settings.min_step), settings.max_step);
    bool previous_rejected = false;
    std::string failure;
    for (std::uint64_t attempts = 0; time_ < end_time; ++attempts) {
        if (attempts >= settings.max_steps) {
            throw std::runtime_error("Adaptive integration exceeded max_steps before reaching the end time");
        }
        // Land exactly on the end time without leaving a sliver of a step.
        const bool last = time_ + 1.01 * h >= end_time;
        const double next_time = last ? end_time : time_ + h;
        const double step = next_time - time_;
        double error = std::numeric_limits<double>::infinity();
        Vector next;
        State projected;
        try {
            advancingStep(time_, step);
            Vector difference;
            std::tie(next, difference) = system.dormandPrince(time_, pack(state_, external_work_), step);
            error = system.errorNorm(difference, pack(state_, external_work_), next, settings);
            if (error <= 1.0) {
                projected = unpack(next);
                unchecked::projectConstraints(model_, projected, next_time);
            }
        } catch (const std::exception& exception) {
            // A singular stage or failed projection: retry with a smaller step.
            failure = exception.what();
            error = std::numeric_limits<double>::infinity();
        }
        if (std::isfinite(error) && error <= 1.0) {
            const double work = next[next.size() - 1];
            accept(std::move(projected), work, next_time);
            double factor = 0.9 * std::pow(std::max(error, 1e-10), -0.17) * std::pow(previous_error_, 0.04);
            factor = std::min(10.0, std::max(0.2, factor));
            if (previous_rejected) factor = std::min(factor, 1.0);
            previous_error_ = std::max(error, 1e-4);
            // A shortened final step says nothing about the sustainable step size.
            h = last ? std::max(h, step * factor) : step * factor;
            h = std::min(h, settings.max_step);
            previous_rejected = false;
        } else {
            ++statistics_.rejected;
            const double factor = std::isfinite(error) ? std::max(0.2, 0.9 * std::pow(error, -0.2)) : 0.25;
            h = step * factor;
            previous_rejected = true;
            if (h < std::max(settings.min_step, 16.0 * std::numeric_limits<double>::epsilon() * std::abs(time_))) {
                throw std::runtime_error("Adaptive step size fell below min_step at t=" + std::to_string(time_)
                    + (failure.empty() ? std::string() : ": " + failure));
            }
        }
    }
    suggested_step_ = h;
}

Diagnostics Simulation::diagnostics() const {
    Diagnostics result;
    result.energy = energy(model_, state_, time_);
    result.energy_change = result.energy.total() - initial_energy_;
    const double scale = std::max(1.0, std::abs(initial_energy_));
    result.scaled_energy_change = result.energy_change / scale;
    result.external_work = external_work_;
    result.energy_balance_error = result.energy_change - external_work_;
    result.scaled_energy_balance_error = result.energy_balance_error / scale;
    result.momentum = momentum(model_, state_);
    result.constraints = constraintError(model_, state_, time_);
    result.steps = statistics_;
    return result;
}
} // namespace physics
