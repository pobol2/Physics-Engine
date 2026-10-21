#include "examples.hpp"

#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace physics::examples {
namespace {
void requireFinite(std::initializer_list<double> values, const char* message) {
    for (double value : values) {
        if (!std::isfinite(value)) throw std::invalid_argument(message);
    }
}
} // namespace

Example drivenOscillator(const DrivenOscillatorConfig& config) {
    requireFinite({config.force_amplitude, config.drive_frequency}, "Drive amplitude and frequency must be finite");
    Model model;
    model.masses = {config.mass};
    model.harmonic_forces = {{0, Eigen::Vector3d::Zero(), config.stiffness}};
    model.damping = {{0, config.damping}};
    model.forces = {harmonicDrive(0, {config.force_amplitude, 0.0, 0.0}, config.drive_frequency)};
    State state{{{config.displacement, 0.0, 0.0}}, {{config.velocity, 0.0, 0.0}}};
    validateModel(model);
    validateState(model, state);
    return {model, state};
}

Example parametricOscillator(const ParametricOscillatorConfig& config) {
    requireFinite({config.mass, config.natural_frequency, config.modulation, config.modulation_frequency},
                  "Parametric oscillator parameters must be finite");
    if (config.mass <= 0.0 || config.natural_frequency <= 0.0 || std::abs(config.modulation) >= 1.0) {
        throw std::invalid_argument("Parametric oscillator needs positive mass and frequency and |modulation| < 1");
    }
    const double k0 = config.mass * config.natural_frequency * config.natural_frequency;
    const double h = config.modulation, gamma = config.modulation_frequency;
    // Stiffness k(t) = k0 (1 + h cos(gamma t)); U = k(t) |q|^2 / 2.
    CustomForce force;
    force.particles = {0};
    force.apply = [=](const SystemView& system, Vectors& forces) {
        forces[0] -= system.state.positions[0] * (k0 * (1.0 + h * std::cos(gamma * system.time)));
    };
    force.potential = [=](const SystemView& system) {
        return 0.5 * k0 * (1.0 + h * std::cos(gamma * system.time)) * system.state.positions[0].squaredNorm();
    };
    force.potential_rate = [=](const SystemView& system) {
        return -0.5 * k0 * h * gamma * std::sin(gamma * system.time) * system.state.positions[0].squaredNorm();
    };
    Model model;
    model.masses = {config.mass};
    model.forces = {customForce(std::move(force))};
    State state{{{config.displacement, 0.0, 0.0}}, {{config.velocity, 0.0, 0.0}}};
    validateModel(model);
    validateState(model, state);
    return {model, state};
}
} // namespace physics::examples
