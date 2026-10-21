#include "physics/constraint.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace physics {
namespace {
void require(bool valid, const char* message) {
    if (!valid) throw std::invalid_argument(message);
}

class PlaneConstraint final : public Constraint {
public:
    PlaneConstraint(std::size_t particle, Motion point, const Eigen::Vector3d& normal,
                    const Eigen::Vector3d& angular_velocity)
        : particle_(particle), point_(std::move(point)), normal_(normal), angular_velocity_(angular_velocity) {}

    void validate(std::size_t count) const override {
        require(particle_ < count, "Plane constraint particle index out of range");
        point_.validate();
        require(normal_.allFinite() && angular_velocity_.allFinite(), "Plane parameters must be finite");
        require(normal_.stableNorm() > 0.0, "Plane normal must be nonzero");
    }

    void appendRows(const SystemView& system, std::vector<ConstraintRow>& rows) const override {
        Eigen::Vector3d n = normal_.normalized();
        const double rate = angular_velocity_.norm();
        if (rate > 0.0) n = Eigen::AngleAxisd(rate * system.time, angular_velocity_ / rate) * n;
        const Eigen::Vector3d n_dot = angular_velocity_.cross(n);
        const Eigen::Vector3d n_ddot = angular_velocity_.cross(n_dot);
        const Eigen::Vector3d offset = system.state.positions[particle_] - point_.position(system.time);
        const Eigen::Vector3d point_velocity = point_.velocity(system.time);
        const Eigen::Vector3d relative_velocity = system.state.velocities[particle_] - point_velocity;
        ConstraintRow row;
        row.terms = {{particle_, n}};
        row.value = n.dot(offset);
        row.velocity_bias = n_dot.dot(offset) - n.dot(point_velocity);
        row.acceleration_bias = -n.dot(point_.acceleration(system.time)) + 2.0 * n_dot.dot(relative_velocity)
            + n_ddot.dot(offset);
        row.scale = std::max(1.0, offset.stableNorm());
        rows.push_back(std::move(row));
    }

private:
    std::size_t particle_;
    Motion point_;
    Eigen::Vector3d normal_;
    Eigen::Vector3d angular_velocity_;
};

class FunctionConstraint final : public Constraint {
public:
    explicit FunctionConstraint(CustomConstraint definition) : definition_(std::move(definition)) {}
    void validate(std::size_t count) const override {
        require(static_cast<bool>(definition_.rows), "Custom constraint needs a rows function");
        for (std::size_t particle : definition_.particles) {
            require(particle < count, "Custom constraint particle index out of range");
        }
    }
    void appendRows(const SystemView& system, std::vector<ConstraintRow>& rows) const override {
        const std::size_t first = rows.size();
        definition_.rows(system, rows);
        for (std::size_t k = first; k < rows.size(); ++k) {
            for (const auto& term : rows[k].terms) {
                bool declared = false;
                for (std::size_t particle : definition_.particles) declared = declared || particle == term.particle;
                require(declared, "Custom constraint row references an undeclared particle");
            }
        }
    }

private:
    CustomConstraint definition_;
};
} // namespace

ConstraintPtr planeConstraint(std::size_t particle, Motion point, const Eigen::Vector3d& normal,
                              const Eigen::Vector3d& angular_velocity) {
    return std::make_shared<PlaneConstraint>(particle, std::move(point), normal, angular_velocity);
}

ConstraintPtr customConstraint(CustomConstraint definition) {
    return std::make_shared<FunctionConstraint>(std::move(definition));
}
} // namespace physics
