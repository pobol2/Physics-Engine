#pragma once

#include <Eigen/Core>

#include <functional>
#include <memory>

namespace physics {
using VectorFunction = std::function<Eigen::Vector3d(double time)>;

// Prescribed trajectory of a support point: position, velocity, and acceleration
// as functions of time. Pivots, anchors, and driven particles take a Motion, so
// any support can move. Motion is an immutable value; copies share custom
// functions, which must therefore be pure.
class Motion {
public:
    // Stationary at the origin.
    Motion() = default;
    // A fixed point converts implicitly, so `{x, y, z}` or a vector can be used
    // wherever a Motion is expected.
    template <class Derived>
    Motion(const Eigen::MatrixBase<Derived>& point) : origin_(point) {}
    Motion(double x, double y, double z) : origin_(x, y, z) {}

    static Motion stationary(const Eigen::Vector3d& point);
    // start + velocity * t
    static Motion linear(const Eigen::Vector3d& start, const Eigen::Vector3d& velocity);
    // center + amplitude * cos(angular_frequency * t + phase)
    static Motion oscillating(const Eigen::Vector3d& center, const Eigen::Vector3d& amplitude,
                              double angular_frequency, double phase = 0.0);
    // center + R(t) * radius, where R(t) rotates by angular_velocity * t about
    // the direction of angular_velocity (right-handed).
    static Motion circular(const Eigen::Vector3d& center, const Eigen::Vector3d& radius,
                           const Eigen::Vector3d& angular_velocity);
    // Arbitrary trajectory. The three functions must be mutually consistent
    // derivatives; the engine does not differentiate numerically.
    static Motion custom(VectorFunction position, VectorFunction velocity, VectorFunction acceleration);

    // Non-finite results throw std::invalid_argument.
    Eigen::Vector3d position(double time) const;
    Eigen::Vector3d velocity(double time) const;
    Eigen::Vector3d acceleration(double time) const;
    bool isStationary() const { return kind_ == Kind::Stationary; }
    // Throws std::invalid_argument for non-finite parameters or missing functions.
    void validate() const;

private:
    enum class Kind { Stationary, Linear, Oscillating, Circular, Custom };
    struct Functions {
        VectorFunction position, velocity, acceleration;
    };
    Kind kind_ = Kind::Stationary;
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();  // point, start, or center
    Eigen::Vector3d vector_ = Eigen::Vector3d::Zero();  // velocity, amplitude, or radius
    Eigen::Vector3d axis_ = Eigen::Vector3d::Zero();    // angular velocity
    double frequency_ = 0.0;
    double phase_ = 0.0;
    std::shared_ptr<const Functions> custom_;
};
} // namespace physics
