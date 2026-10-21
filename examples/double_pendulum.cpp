#include "examples.hpp"

#include <cmath>
#include <stdexcept>

namespace physics::examples {
Example doublePendulum(const DoublePendulumConfig& config) {
    for (double value : {config.gravity, config.angle1, config.angle2, config.angular_velocity1,
                         config.angular_velocity2}) {
        if (!std::isfinite(value)) throw std::invalid_argument("Double pendulum parameters must be finite");
    }
    if (config.gravity < 0.0) throw std::invalid_argument("Gravity must be nonnegative");
    Model model;
    model.masses = {config.mass1, config.mass2};
    model.gravity = {0.0, -config.gravity, 0.0};
    model.distance_constraints = {{0, Eigen::Vector3d::Zero(), config.length1}};
    model.links = {{0, 1, config.length2}};
    const auto offset = [](double length, double angle) {
        return Eigen::Vector3d{length * std::sin(angle), -length * std::cos(angle), 0.0};
    };
    const auto rate = [](double length, double angle, double omega) {
        return Eigen::Vector3d{length * omega * std::cos(angle), length * omega * std::sin(angle), 0.0};
    };
    const Eigen::Vector3d q1 = offset(config.length1, config.angle1);
    const Eigen::Vector3d v1 = rate(config.length1, config.angle1, config.angular_velocity1);
    State state{{q1, q1 + offset(config.length2, config.angle2)},
                {v1, v1 + rate(config.length2, config.angle2, config.angular_velocity2)}};
    validateModel(model);
    validateState(model, state);
    return {model, state};
}
} // namespace physics::examples
