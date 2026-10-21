#pragma once

#include "physics/motion.hpp"
#include "physics/state.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

namespace physics {
// Extension point for applied forces. Implementations are immutable and shared
// between copies of a Model, so they must not keep mutable state.
class Force {
public:
    virtual ~Force() = default;
    // Throw std::invalid_argument if the configuration is invalid for a system of
    // particle_count particles (bad indices, non-finite or out-of-range values).
    virtual void validate(std::size_t particle_count) const = 0;
    // Add this interaction's force on each particle to `forces` (one entry per
    // particle). Add only; never clear or overwrite other contributions.
    virtual void accumulate(const SystemView& system, Vectors& forces) const = 0;
    // A conservative force derives from a potential U(q, t) with F = -grad U. Its
    // potential is part of Energy::potential. The power of nonconservative forces
    // (drives, drag, magnetic forces) is booked as external work instead.
    virtual bool conservative() const { return false; }
    virtual double potential(const SystemView& system) const;
    // Explicit time dependence: partial dU/dt at fixed positions.
    virtual double potentialRate(const SystemView& system) const;
};
using ForcePtr = std::shared_ptr<const Force>;

// Field evaluated at a position and time, e.g. an electric or magnetic field.
using FieldFunction = std::function<Eigen::Vector3d(const Eigen::Vector3d& position, double time)>;
FieldFunction uniformField(const Eigen::Vector3d& value);

// Radial potential U(r) and its derivative dU/dr, for separations r > 0.
struct RadialPotential {
    std::function<double(double)> energy;
    std::function<double(double)> derivative;
};
// U = -strength / r. Attractive for strength > 0: Newtonian gravity with
// strength = G m1 m2, or Coulomb attraction. Negative strength repels.
RadialPotential inverseSquare(double strength);
// U = 4 depth ((sigma / r)^12 - (sigma / r)^6).
RadialPotential lennardJones(double depth, double sigma);

// Explicitly time-dependent external force F(t) on one particle.
ForcePtr drivingForce(std::size_t particle, VectorFunction force);
// F(t) = amplitude * cos(angular_frequency * t + phase).
ForcePtr harmonicDrive(std::size_t particle, const Eigen::Vector3d& amplitude,
                       double angular_frequency, double phase = 0.0);
// Central field U(|q - center(t)|) around a possibly moving center.
ForcePtr centralForce(std::size_t particle, Motion center, RadialPotential potential);
// Interaction U(|q_first - q_second|) between two particles.
ForcePtr pairForce(std::size_t first, std::size_t second, RadialPotential potential);
// Newtonian gravity between every pair of the listed particles (all particles
// when empty), using the model masses: U = -G m_i m_j / sqrt(r^2 + softening^2).
ForcePtr gravitation(double constant, std::vector<std::size_t> particles = {}, double softening = 0.0);
// F = -coefficient * |v| * v.
ForcePtr quadraticDrag(std::size_t particle, double coefficient);
// F = charge * (E(q, t) + v x B(q, t)). Either field may be empty.
ForcePtr lorentzForce(std::size_t particle, double charge, FieldFunction electric, FieldFunction magnetic);

// Force defined by functions, for interactions without a dedicated class.
struct CustomForce {
    // Every particle the functions read or write, checked against the model.
    std::vector<std::size_t> particles;
    // Adds forces; same contract as Force::accumulate.
    std::function<void(const SystemView&, Vectors&)> apply;
    // Set for conservative forces; leave empty otherwise.
    std::function<double(const SystemView&)> potential;
    // Partial dU/dt of a time-dependent potential; empty means zero.
    std::function<double(const SystemView&)> potential_rate;
};
ForcePtr customForce(CustomForce definition);
} // namespace physics
