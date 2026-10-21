#include "examples.hpp"

namespace physics::examples {
Example oscillator(const OscillatorConfig& config) {
    Model model;
    model.masses = {config.mass};
    model.harmonic_forces = {{0, Eigen::Vector3d::Zero(), config.stiffness}};
    model.damping = {{0, config.damping}};
    State state{{{config.displacement, 0.0, 0.0}}, {{config.velocity, 0.0, 0.0}}};
    validateModel(model);
    validateState(model, state);
    return {model, state};
}
} // namespace physics::examples
