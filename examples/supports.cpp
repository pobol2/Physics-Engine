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

Eigen::Vector3d bob(double length, double angle) {
    return {length * std::sin(angle), -length * std::cos(angle), 0.0};
}

Eigen::Vector3d tangent(double length, double angle, double angular_velocity) {
    return {length * angular_velocity * std::cos(angle), length * angular_velocity * std::sin(angle), 0.0};
}
} // namespace

Example kapitzaPendulum(const KapitzaPendulumConfig& config) {
    requireFinite({config.gravity, config.pivot_amplitude, config.pivot_frequency, config.angle,
                   config.angular_velocity}, "Kapitza pendulum parameters must be finite");
    if (config.gravity < 0.0) throw std::invalid_argument("Gravity must be nonnegative");
    const Motion pivot = Motion::oscillating(Eigen::Vector3d::Zero(), {0.0, config.pivot_amplitude, 0.0},
                                             config.pivot_frequency);
    Model model;
    model.masses = {config.mass};
    model.gravity = {0.0, -config.gravity, 0.0};
    model.distance_constraints = {{0, pivot, config.length}};
    State state{{pivot.position(0.0) + bob(config.length, config.angle)},
                {pivot.velocity(0.0) + tangent(config.length, config.angle, config.angular_velocity)}};
    validateModel(model);
    validateState(model, state);
    return {model, state};
}

Example rotatingHoop(const RotatingHoopConfig& config) {
    requireFinite({config.gravity, config.rotation_rate, config.angle, config.angular_velocity},
                  "Rotating hoop parameters must be finite");
    if (config.gravity < 0.0) throw std::invalid_argument("Gravity must be nonnegative");
    // The hoop lies in the plane spanned by y and u(t) = R_y(Omega t) x; its
    // normal z rotates about the vertical y axis.
    Model model;
    model.masses = {config.mass};
    model.gravity = {0.0, -config.gravity, 0.0};
    model.distance_constraints = {{0, Eigen::Vector3d::Zero(), config.radius}};
    model.constraints = {planeConstraint(0, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
                                         {0.0, config.rotation_rate, 0.0})};
    const Eigen::Vector3d position = bob(config.radius, config.angle);
    // Velocity within the hoop plus the hoop's rotation, Omega y x q.
    const Eigen::Vector3d velocity = tangent(config.radius, config.angle, config.angular_velocity)
        + Eigen::Vector3d{0.0, 0.0, -config.rotation_rate * position.x()};
    State state{{position}, {velocity}};
    validateModel(model);
    validateState(model, state);
    return {model, state};
}
} // namespace physics::examples
