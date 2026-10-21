#pragma once

#include "physics/motion.hpp"
#include "physics/state.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

namespace physics {
struct ConstraintTerm {
    std::size_t particle = 0;
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero(); // dC/dq for this particle
};

// One scalar holonomic constraint C(q, t) = 0, linearized at the evaluation
// state. The solver needs three levels:
//   position:     C = value                              (Newton projection)
//   velocity:     dC/dt = J v + velocity_bias             (velocity_bias = partial dC/dt)
//   acceleration: J a + acceleration_bias = 0            (Lagrange multipliers)
// acceleration_bias may be any expression equal to dJ/dt v + d(partial dC/dt)/dt
// on the constraint manifold; only that equation is enforced at RK stages.
struct ConstraintRow {
    std::vector<ConstraintTerm> terms;
    double value = 0.0;
    double velocity_bias = 0.0;
    double acceleration_bias = 0.0;
    // Characteristic size of C (e.g. a rod length) for convergence tolerances.
    double scale = 1.0;
};

// Extension point for ideal holonomic constraints. Implementations are
// immutable and shared between copies of a Model.
class Constraint {
public:
    virtual ~Constraint() = default;
    virtual void validate(std::size_t particle_count) const = 0;
    // Append this constraint's rows evaluated at the supplied state and time.
    // Throw std::invalid_argument where the geometry is singular.
    virtual void appendRows(const SystemView& system, std::vector<ConstraintRow>& rows) const = 0;
};
using ConstraintPtr = std::shared_ptr<const Constraint>;

// Keeps a particle on the plane through point(t) whose unit normal starts as
// `normal` and rotates at constant angular_velocity. Rotating or translating
// planes are rheonomic: they do work on the particle.
ConstraintPtr planeConstraint(std::size_t particle, Motion point, const Eigen::Vector3d& normal,
                              const Eigen::Vector3d& angular_velocity = Eigen::Vector3d::Zero());

// Constraint defined by a function, for geometry without a dedicated class.
struct CustomConstraint {
    // Every particle the rows may reference, checked against the model.
    std::vector<std::size_t> particles;
    std::function<void(const SystemView&, std::vector<ConstraintRow>&)> rows;
};
ConstraintPtr customConstraint(CustomConstraint definition);
} // namespace physics
