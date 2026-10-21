#include "physics/motion.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace physics {
namespace {
void require(bool valid, const char* message) {
    if (!valid) throw std::invalid_argument(message);
}

Eigen::Vector3d finite(const Eigen::Vector3d& value, const char* message) {
    require(value.allFinite(), message);
    return value;
}
} // namespace

Motion Motion::stationary(const Eigen::Vector3d& point) {
    return Motion(point);
}

Motion Motion::linear(const Eigen::Vector3d& start, const Eigen::Vector3d& velocity) {
    Motion motion(start);
    motion.kind_ = Kind::Linear;
    motion.vector_ = velocity;
    return motion;
}

Motion Motion::oscillating(const Eigen::Vector3d& center, const Eigen::Vector3d& amplitude,
                           double angular_frequency, double phase) {
    Motion motion(center);
    motion.kind_ = Kind::Oscillating;
    motion.vector_ = amplitude;
    motion.frequency_ = angular_frequency;
    motion.phase_ = phase;
    return motion;
}

Motion Motion::circular(const Eigen::Vector3d& center, const Eigen::Vector3d& radius,
                        const Eigen::Vector3d& angular_velocity) {
    Motion motion(center);
    motion.kind_ = Kind::Circular;
    motion.vector_ = radius;
    motion.axis_ = angular_velocity;
    return motion;
}

Motion Motion::custom(VectorFunction position, VectorFunction velocity, VectorFunction acceleration) {
    Motion motion;
    motion.kind_ = Kind::Custom;
    motion.custom_ = std::make_shared<const Functions>(
        Functions{std::move(position), std::move(velocity), std::move(acceleration)});
    return motion;
}

void Motion::validate() const {
    require(origin_.allFinite() && vector_.allFinite() && axis_.allFinite()
                && std::isfinite(frequency_) && std::isfinite(phase_),
            "Motion parameters must be finite");
    if (kind_ == Kind::Custom) {
        require(custom_ && custom_->position && custom_->velocity && custom_->acceleration,
                "Custom motion needs position, velocity, and acceleration functions");
    }
}

Eigen::Vector3d Motion::position(double time) const {
    switch (kind_) {
    case Kind::Stationary: return origin_;
    case Kind::Linear: return finite(origin_ + vector_ * time, "Motion position must be finite");
    case Kind::Oscillating:
        return finite(origin_ + vector_ * std::cos(frequency_ * time + phase_), "Motion position must be finite");
    case Kind::Circular: {
        const double rate = axis_.norm();
        if (rate == 0.0) return origin_ + vector_;
        const Eigen::AngleAxisd rotation(rate * time, axis_ / rate);
        return finite(origin_ + rotation * vector_, "Motion position must be finite");
    }
    case Kind::Custom: return finite(custom_->position(time), "Motion position must be finite");
    }
    throw std::logic_error("Unknown motion kind");
}

Eigen::Vector3d Motion::velocity(double time) const {
    switch (kind_) {
    case Kind::Stationary: return Eigen::Vector3d::Zero();
    case Kind::Linear: return vector_;
    case Kind::Oscillating:
        return finite(vector_ * (-frequency_ * std::sin(frequency_ * time + phase_)),
                      "Motion velocity must be finite");
    case Kind::Circular: return finite(axis_.cross(position(time) - origin_), "Motion velocity must be finite");
    case Kind::Custom: return finite(custom_->velocity(time), "Motion velocity must be finite");
    }
    throw std::logic_error("Unknown motion kind");
}

Eigen::Vector3d Motion::acceleration(double time) const {
    switch (kind_) {
    case Kind::Stationary:
    case Kind::Linear: return Eigen::Vector3d::Zero();
    case Kind::Oscillating:
        return finite(vector_ * (-frequency_ * frequency_ * std::cos(frequency_ * time + phase_)),
                      "Motion acceleration must be finite");
    case Kind::Circular:
        return finite(axis_.cross(axis_.cross(position(time) - origin_)), "Motion acceleration must be finite");
    case Kind::Custom: return finite(custom_->acceleration(time), "Motion acceleration must be finite");
    }
    throw std::logic_error("Unknown motion kind");
}
} // namespace physics
