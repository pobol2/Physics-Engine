#include "examples.hpp"

#include <cmath>
#include <stdexcept>

namespace physics::examples {
Example pendulum(const PendulumConfig& config) {
    if (!std::isfinite(config.gravity) || config.gravity <= 0.0
        || !std::isfinite(config.angle) || !std::isfinite(config.angular_velocity)) {
        throw std::invalid_argument("Pendulum gravity must be positive; angles and angular velocity must be finite");
    }
    Model model;
    model.masses = {config.mass};
    model.gravity = {0.0, -config.gravity, 0.0};
    model.distance_constraints = {{0, Eigen::Vector3d::Zero(), config.length}};
    model.damping = {{0, config.damping}};
    State state{{{config.length * std::sin(config.angle), -config.length * std::cos(config.angle), 0.0}},
                {{config.length * config.angular_velocity * std::cos(config.angle),
                  config.length * config.angular_velocity * std::sin(config.angle), 0.0}}};
    validateModel(model);
    validateState(model, state);
    return {model, state};
}
} // namespace physics::examples
