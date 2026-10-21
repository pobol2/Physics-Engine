#pragma once

#include "physics/constraint.hpp"
#include "physics/force.hpp"
#include "physics/motion.hpp"
#include "physics/state.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace physics {
// Configuration is separate from state. Simulation owns a validated copy and
// exposes it as const; evaluation never accumulates forces or changes the state.

// F = -stiffness (q - equilibrium(t)); the equilibrium may move.
struct HarmonicForce {
    std::size_t particle = 0;
    Motion equilibrium;
    double stiffness = 0.0;
};

struct PairSpring {
    std::size_t first = 0;
    std::size_t second = 0;
    double stiffness = 0.0;
    double rest_length = 0.0;
};

struct LinearDamping {
    std::size_t particle = 0;
    double coefficient = 0.0; // kg/s; force = -coefficient * velocity
};

// The particle follows `motion` exactly (a fixed or moving support). Its mass
// does not affect the other particles; it still counts in energy and momentum.
struct PrescribedMotion {
    std::size_t particle = 0;
    Motion motion;
};

// Massless rigid rod from a (possibly moving) pivot: |q - pivot(t)| = length.
struct DistanceConstraint {
    std::size_t particle = 0;
    Motion pivot;
    double length = 0.0;
};

// Massless rigid rod between two particles: |q_first - q_second| = length.
struct LinkConstraint {
    std::size_t first = 0;
    std::size_t second = 0;
    double length = 0.0;
};

struct Model {
    std::vector<double> masses;
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
    // Built-in forces.
    std::vector<HarmonicForce> harmonic_forces;
    std::vector<PairSpring> springs;
    std::vector<LinearDamping> damping;
    // Reusable and user-defined forces (see force.hpp).
    std::vector<ForcePtr> forces;
    // Constraints. Any number may act on one particle; they are solved together.
    std::vector<PrescribedMotion> prescribed;
    std::vector<DistanceConstraint> distance_constraints;
    std::vector<LinkConstraint> links;
    std::vector<ConstraintPtr> constraints; // see constraint.hpp
};

struct Energy {
    double kinetic = 0.0;
    double potential = 0.0;
    double total() const { return kinetic + potential; }
};

struct Momentum {
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero(); // about the origin
};

struct ConstraintError {
    double max_position_error = 0.0; // max |C| over constraint rows and prescribed particles
    double max_velocity_error = 0.0; // max |dC/dt|
};

struct Dynamics {
    Vectors forces;        // applied forces, excluding constraint reactions
    Vectors accelerations; // including constraint reactions and prescribed motion
    // dE/dt along the equations of motion, E = T + U: power of nonconservative
    // forces, constraint reactions, prescribed supports, and explicit dU/dt.
    double external_power = 0.0;
};

void validateModel(const Model& model);
// Public evaluation functions validate configuration and state on every call.
void validateState(const Model& model, const State& state);
// Fresh applied forces at the supplied state and time, excluding reactions.
Vectors evaluateForces(const Model& model, const State& state, double time);
// Accelerations satisfying every constraint at acceleration level. Prescribed
// particles get their motion's acceleration.
Vectors evaluateAccelerations(const Model& model, const State& state, double time);
Dynamics evaluateDynamics(const Model& model, const State& state, double time);
// Kinetic energy plus the potential of all conservative forces at `time`.
Energy energy(const Model& model, const State& state, double time);
Momentum momentum(const Model& model, const State& state);
ConstraintError constraintError(const Model& model, const State& state, double time);
// Sets prescribed particles to their motion at `time`.
void applyPrescribedMotion(const Model& model, State& state, double time);
// Applies prescribed motion, then moves positions onto the constraint manifold by
// mass-weighted Newton iteration and removes constraint-violating velocity.
// Transactional: on failure (singular or inconsistent constraints) it throws
// std::invalid_argument and leaves the supplied state unchanged.
void projectConstraints(const Model& model, State& state, double time);
} // namespace physics
