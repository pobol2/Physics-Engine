#include "physics/model.hpp"

#include <Eigen/Geometry>
#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace physics {
namespace {
void require(bool valid, const char* message) {
    if (!valid) throw std::invalid_argument(message);
}

double springLength(const PairSpring& spring, const State& state) {
    const double length = (state.positions.at(spring.first) - state.positions.at(spring.second)).stableNorm();
    require(std::isfinite(length), "Spring separation must be finite");
    require(length > 0.0 || spring.rest_length == 0.0,
            "Positive-rest-length spring cannot have coincident endpoints");
    return length;
}

void validateInputs(const Model& model, const State& state, double time) {
    validateModel(model);
    validateState(model, state);
    require(std::isfinite(time), "Evaluation time must be finite");
}

// Inverse masses, zero for prescribed particles: constraint reactions never move them.
std::vector<double> inverseMasses(const Model& model) {
    std::vector<double> result(model.masses.size());
    for (std::size_t i = 0; i < result.size(); ++i) result[i] = 1.0 / model.masses[i];
    for (const auto& prescribed : model.prescribed) result[prescribed.particle] = 0.0;
    return result;
}

struct AppliedForces {
    Vectors conservative;
    Vectors nonconservative;
    double potential_rate = 0.0;
};

AppliedForces appliedForces(const Model& model, const State& state, double time) {
    const std::size_t n = model.masses.size();
    AppliedForces result{Vectors(n, Eigen::Vector3d::Zero()), Vectors(n, Eigen::Vector3d::Zero())};
    for (std::size_t i = 0; i < n; ++i) result.conservative[i] = model.gravity * model.masses[i];
    for (const auto& force : model.harmonic_forces) {
        const Eigen::Vector3d offset = state.positions[force.particle] - force.equilibrium.position(time);
        result.conservative[force.particle] -= offset * force.stiffness;
        if (!force.equilibrium.isStationary()) {
            result.potential_rate -= force.stiffness * offset.dot(force.equilibrium.velocity(time));
        }
    }
    for (const auto& spring : model.springs) {
        const Eigen::Vector3d d = state.positions[spring.first] - state.positions[spring.second];
        const double length = springLength(spring, state);
        // Zero rest length has the smooth limit F=-k*d, including d=0.
        Eigen::Vector3d force = d * (-spring.stiffness);
        if (spring.rest_length != 0.0) {
            force = (d / length) * (-spring.stiffness * (length - spring.rest_length));
        }
        result.conservative[spring.first] += force;
        result.conservative[spring.second] -= force;
    }
    for (const auto& damping : model.damping) {
        result.nonconservative[damping.particle] -= state.velocities[damping.particle] * damping.coefficient;
    }
    const SystemView view{model.masses, state, time};
    for (const auto& force : model.forces) {
        if (force->conservative()) {
            force->accumulate(view, result.conservative);
            result.potential_rate += force->potentialRate(view);
        } else {
            force->accumulate(view, result.nonconservative);
        }
    }
    require(result.conservative.size() == n && result.nonconservative.size() == n,
            "A force changed the size of the force buffer");
    for (std::size_t i = 0; i < n; ++i) {
        require(result.conservative[i].allFinite() && result.nonconservative[i].allFinite(),
                "Evaluated force must be finite");
    }
    require(std::isfinite(result.potential_rate), "Potential rate must be finite");
    return result;
}

// ---- Constraint rows and the multiplier solver ------------------------------

ConstraintRow distanceRow(std::size_t first, const Eigen::Vector3d& d, const Eigen::Vector3d& relative_velocity,
                          double length) {
    const double r = d.stableNorm();
    require(std::isfinite(r) && r > 0.0, "Constrained particle cannot be at its pivot");
    const Eigen::Vector3d direction = d / r;
    ConstraintRow row;
    row.terms = {{first, direction}};
    row.value = r - length;
    // From d/dt^2 (|d|^2 / 2) = d.a_rel + |v_rel|^2 = 0, divided by r.
    row.acceleration_bias = relative_velocity.squaredNorm() / r;
    row.scale = length;
    return row;
}

std::vector<ConstraintRow> collectRows(const Model& model, const State& state, double time) {
    std::vector<ConstraintRow> rows;
    for (const auto& constraint : model.distance_constraints) {
        const auto i = constraint.particle;
        const Eigen::Vector3d pivot_velocity = constraint.pivot.velocity(time);
        ConstraintRow row = distanceRow(i, state.positions[i] - constraint.pivot.position(time),
                                        state.velocities[i] - pivot_velocity, constraint.length);
        const Eigen::Vector3d& direction = row.terms[0].gradient;
        row.velocity_bias = -direction.dot(pivot_velocity);
        row.acceleration_bias -= direction.dot(constraint.pivot.acceleration(time));
        rows.push_back(std::move(row));
    }
    for (const auto& link : model.links) {
        ConstraintRow row = distanceRow(link.first, state.positions[link.first] - state.positions[link.second],
                                        state.velocities[link.first] - state.velocities[link.second], link.length);
        row.terms.push_back({link.second, -row.terms[0].gradient});
        require(std::isfinite(row.value), "Link separation must be finite");
        rows.push_back(std::move(row));
    }
    const SystemView view{model.masses, state, time};
    for (const auto& constraint : model.constraints) constraint->appendRows(view, rows);
    for (const auto& row : rows) {
        require(std::isfinite(row.value) && std::isfinite(row.velocity_bias) && std::isfinite(row.acceleration_bias)
                    && std::isfinite(row.scale) && row.scale > 0.0,
                "Constraint row values must be finite with a positive scale");
        for (const auto& term : row.terms) {
            require(term.particle < model.masses.size(), "Constraint row particle index out of range");
            require(term.gradient.allFinite(), "Constraint gradient must be finite");
        }
    }
    return rows;
}

double rowProduct(const ConstraintRow& row, const Vectors& values) {
    double result = 0.0;
    for (const auto& term : row.terms) result += term.gradient.dot(values[term.particle]);
    return result;
}

std::size_t findRoot(std::vector<std::size_t>& parent, std::size_t i) {
    while (parent[i] != i) i = parent[i] = parent[parent[i]];
    return i;
}

// Solves (J W J^T) lambda = rhs, where W holds inverse masses. Rows that share no
// free particle are independent, so each coupled block is solved separately:
// single rows exactly, larger blocks by complete orthogonal decomposition, which
// gives the minimum-norm solution for redundant constraints.
Eigen::VectorXd solveMultipliers(const std::vector<ConstraintRow>& rows, const std::vector<double>& inverse_mass,
                                 const Eigen::VectorXd& rhs) {
    const std::size_t particle_count = inverse_mass.size();
    std::vector<std::size_t> parent(particle_count);
    std::iota(parent.begin(), parent.end(), std::size_t{0});
    constexpr std::size_t none = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> anchor(rows.size(), none);
    for (std::size_t k = 0; k < rows.size(); ++k) {
        for (const auto& term : rows[k].terms) {
            if (inverse_mass[term.particle] == 0.0) continue;
            if (anchor[k] == none) anchor[k] = term.particle;
            else parent[findRoot(parent, term.particle)] = findRoot(parent, anchor[k]);
        }
    }
    std::vector<std::vector<std::size_t>> blocks(particle_count);
    for (std::size_t k = 0; k < rows.size(); ++k) {
        if (anchor[k] != none) blocks[findRoot(parent, anchor[k])].push_back(k);
    }
    const auto coupling = [&](const ConstraintRow& a, const ConstraintRow& b) {
        double sum = 0.0;
        for (const auto& s : a.terms) {
            for (const auto& t : b.terms) {
                if (s.particle == t.particle) sum += inverse_mass[s.particle] * s.gradient.dot(t.gradient);
            }
        }
        return sum;
    };
    Eigen::VectorXd lambda = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(rows.size()));
    for (const auto& block : blocks) {
        if (block.empty()) continue;
        if (block.size() == 1) {
            const double a = coupling(rows[block[0]], rows[block[0]]);
            if (a > 0.0) lambda[block[0]] = rhs[block[0]] / a;
            continue;
        }
        const auto m = static_cast<Eigen::Index>(block.size());
        Eigen::MatrixXd a(m, m);
        Eigen::VectorXd b(m);
        for (Eigen::Index i = 0; i < m; ++i) {
            b[i] = rhs[block[i]];
            for (Eigen::Index j = 0; j <= i; ++j) a(i, j) = a(j, i) = coupling(rows[block[i]], rows[block[j]]);
        }
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> solver;
        solver.setThreshold(1e-12);
        solver.compute(a);
        const Eigen::VectorXd x = solver.solve(b);
        for (Eigen::Index i = 0; i < m; ++i) lambda[block[i]] = x[i];
    }
    require(lambda.allFinite(), "Constraint multipliers must be finite");
    return lambda;
}

// target_i += sign * W_i sum_k lambda_k dC_k/dq_i
void applyCorrection(const std::vector<ConstraintRow>& rows, const std::vector<double>& inverse_mass,
                     const Eigen::VectorXd& lambda, double sign, Vectors& target) {
    for (std::size_t k = 0; k < rows.size(); ++k) {
        for (const auto& term : rows[k].terms) {
            target[term.particle] += term.gradient * (sign * inverse_mass[term.particle] * lambda[k]);
        }
    }
}

// Constraint residual relative to the row scale and the coordinates involved,
// so tolerances are meaningful for both small and large coordinates.
double relativeResidual(const std::vector<ConstraintRow>& rows, const State& state) {
    double result = 0.0;
    for (const auto& row : rows) {
        double size = row.scale;
        for (const auto& term : row.terms) size = std::max(size, state.positions[term.particle].stableNorm());
        result = std::max(result, std::abs(row.value) / size);
    }
    return result;
}
} // namespace

void validateModel(const Model& model) {
    const std::size_t n = model.masses.size();
    require(n > 0, "A model needs at least one particle");
    require(model.gravity.allFinite(), "Gravity must be finite");
    for (double mass : model.masses) {
        require(std::isfinite(mass) && mass > 0.0, "Mass must be finite and positive");
    }
    for (const auto& force : model.harmonic_forces) {
        require(force.particle < n, "Invalid harmonic-force particle index");
        force.equilibrium.validate();
        require(std::isfinite(force.stiffness) && force.stiffness > 0.0,
                "Stiffness must be finite and positive");
    }
    for (const auto& spring : model.springs) {
        require(spring.first < n && spring.second < n, "Invalid spring particle index");
        require(spring.first != spring.second, "A spring needs two distinct particles");
        require(std::isfinite(spring.stiffness) && spring.stiffness > 0.0,
                "Spring stiffness must be finite and positive");
        require(std::isfinite(spring.rest_length) && spring.rest_length >= 0.0,
                "Spring rest length must be finite and nonnegative");
    }
    for (const auto& damping : model.damping) {
        require(damping.particle < n, "Invalid damping particle index");
        require(std::isfinite(damping.coefficient) && damping.coefficient >= 0.0,
                "Linear damping must be finite and nonnegative");
    }
    for (const auto& force : model.forces) {
        require(static_cast<bool>(force), "Force pointer cannot be null");
        force->validate(n);
    }
    std::vector<bool> prescribed(n, false);
    for (const auto& motion : model.prescribed) {
        require(motion.particle < n, "Invalid prescribed-motion particle index");
        require(!prescribed[motion.particle], "A particle can follow only one prescribed motion");
        motion.motion.validate();
        prescribed[motion.particle] = true;
    }
    for (const auto& constraint : model.distance_constraints) {
        require(constraint.particle < n, "Invalid distance-constraint particle index");
        require(!prescribed[constraint.particle], "A prescribed particle cannot also have a distance constraint");
        constraint.pivot.validate();
        require(std::isfinite(constraint.length) && constraint.length > 0.0,
                "Constraint length must be finite and positive");
    }
    for (const auto& link : model.links) {
        require(link.first < n && link.second < n, "Invalid link particle index");
        require(link.first != link.second, "A link needs two distinct particles");
        require(!prescribed[link.first] || !prescribed[link.second], "A link needs at least one free particle");
        require(std::isfinite(link.length) && link.length > 0.0, "Link length must be finite and positive");
    }
    for (const auto& constraint : model.constraints) {
        require(static_cast<bool>(constraint), "Constraint pointer cannot be null");
        constraint->validate(n);
    }
}

void validateState(const Model& model, const State& state) {
    require(state.positions.size() == model.masses.size()
                && state.velocities.size() == model.masses.size(), "State dimensions must match model");
    for (std::size_t i = 0; i < model.masses.size(); ++i) {
        require(state.positions[i].allFinite() && state.velocities[i].allFinite(), "State must be finite");
    }
}

Vectors evaluateForces(const Model& model, const State& state, double time) {
    validateInputs(model, state, time);
    auto forces = appliedForces(model, state, time);
    for (std::size_t i = 0; i < forces.conservative.size(); ++i) forces.conservative[i] += forces.nonconservative[i];
    return forces.conservative;
}

Dynamics evaluateDynamics(const Model& model, const State& state, double time) {
    validateInputs(model, state, time);
    const std::size_t n = model.masses.size();
    const auto applied = appliedForces(model, state, time);
    Dynamics result{Vectors(n), Vectors(n)};
    for (std::size_t i = 0; i < n; ++i) {
        result.forces[i] = applied.conservative[i] + applied.nonconservative[i];
        result.accelerations[i] = result.forces[i] / model.masses[i];
    }
    for (const auto& prescribed : model.prescribed) {
        result.accelerations[prescribed.particle] = prescribed.motion.acceleration(time);
    }
    const auto rows = collectRows(model, state, time);
    if (!rows.empty()) {
        const auto inverse_mass = inverseMasses(model);
        Eigen::VectorXd rhs(static_cast<Eigen::Index>(rows.size()));
        for (std::size_t k = 0; k < rows.size(); ++k) {
            rhs[k] = -(rowProduct(rows[k], result.accelerations) + rows[k].acceleration_bias);
        }
        applyCorrection(rows, inverse_mass, solveMultipliers(rows, inverse_mass, rhs), 1.0, result.accelerations);
    }
    // dE/dt = sum (m a - F_conservative) . v + partial dU/dt.
    result.external_power = applied.potential_rate;
    for (std::size_t i = 0; i < n; ++i) {
        require(result.accelerations[i].allFinite(), "Evaluated acceleration must be finite");
        result.external_power += (result.accelerations[i] * model.masses[i] - applied.conservative[i])
            .dot(state.velocities[i]);
    }
    require(std::isfinite(result.external_power), "External power must be finite");
    return result;
}

Vectors evaluateAccelerations(const Model& model, const State& state, double time) {
    return evaluateDynamics(model, state, time).accelerations;
}

Energy energy(const Model& model, const State& state, double time) {
    validateInputs(model, state, time);
    Energy result;
    for (std::size_t i = 0; i < model.masses.size(); ++i) {
        result.kinetic += 0.5 * model.masses[i] * state.velocities[i].dot(state.velocities[i]);
        result.potential -= model.masses[i] * model.gravity.dot(state.positions[i]);
    }
    for (const auto& force : model.harmonic_forces) {
        const Eigen::Vector3d d = state.positions[force.particle] - force.equilibrium.position(time);
        result.potential += 0.5 * force.stiffness * d.dot(d);
    }
    for (const auto& spring : model.springs) {
        const double extension = springLength(spring, state) - spring.rest_length;
        result.potential += 0.5 * spring.stiffness * extension * extension;
    }
    const SystemView view{model.masses, state, time};
    for (const auto& force : model.forces) {
        if (force->conservative()) result.potential += force->potential(view);
    }
    // Damping and other nonconservative forces contribute no potential energy.
    require(std::isfinite(result.kinetic) && std::isfinite(result.potential)
                && std::isfinite(result.total()), "Evaluated energy must be finite");
    return result;
}

Momentum momentum(const Model& model, const State& state) {
    validateModel(model);
    validateState(model, state);
    Momentum result;
    for (std::size_t i = 0; i < model.masses.size(); ++i) {
        const Eigen::Vector3d p = state.velocities[i] * model.masses[i];
        result.linear += p;
        result.angular += state.positions[i].cross(p);
    }
    return result;
}

ConstraintError constraintError(const Model& model, const State& state, double time) {
    validateInputs(model, state, time);
    ConstraintError result;
    for (const auto& prescribed : model.prescribed) {
        const auto i = prescribed.particle;
        result.max_position_error = std::max(result.max_position_error,
            (state.positions[i] - prescribed.motion.position(time)).stableNorm());
        result.max_velocity_error = std::max(result.max_velocity_error,
            (state.velocities[i] - prescribed.motion.velocity(time)).stableNorm());
    }
    for (const auto& row : collectRows(model, state, time)) {
        result.max_position_error = std::max(result.max_position_error, std::abs(row.value));
        result.max_velocity_error = std::max(result.max_velocity_error,
            std::abs(rowProduct(row, state.velocities) + row.velocity_bias));
    }
    return result;
}

void applyPrescribedMotion(const Model& model, State& state, double time) {
    for (const auto& prescribed : model.prescribed) {
        state.positions.at(prescribed.particle) = prescribed.motion.position(time);
        state.velocities.at(prescribed.particle) = prescribed.motion.velocity(time);
    }
}

void projectConstraints(const Model& model, State& state, double time) {
    validateInputs(model, state, time);
    State projected = state;
    applyPrescribedMotion(model, projected, time);
    const auto inverse_mass = inverseMasses(model);
    // Newton iteration on C(q) = 0 with the mass-weighted minimum-norm update
    // dq = -W J^T (J W J^T)^-1 C. One iteration is exact for an isolated rod.
    constexpr int max_iterations = 50;
    // Iterate to rounding level; stop early only once progress stalls there.
    constexpr double tight = 4.0 * std::numeric_limits<double>::epsilon(), loose = 1e-9;
    auto rows = collectRows(model, projected, time);
    if (rows.empty()) {
        validateState(model, projected);
        state = std::move(projected);
        return;
    }
    double residual = relativeResidual(rows, projected);
    double previous = std::numeric_limits<double>::infinity();
    for (int iteration = 0; iteration < max_iterations && residual > tight; ++iteration) {
        // Accept a residual that has stopped improving at the rounding level.
        if (residual <= loose && residual > 0.5 * previous) break;
        Eigen::VectorXd rhs(static_cast<Eigen::Index>(rows.size()));
        for (std::size_t k = 0; k < rows.size(); ++k) rhs[k] = rows[k].value;
        applyCorrection(rows, inverse_mass, solveMultipliers(rows, inverse_mass, rhs), -1.0, projected.positions);
        validateState(model, projected);
        rows = collectRows(model, projected, time);
        previous = residual;
        residual = relativeResidual(rows, projected);
    }
    require(residual <= loose, "Constraint projection did not converge: constraints are inconsistent or singular");
    // Velocity constraints are linear: one solve removes the violating component.
    Eigen::VectorXd rhs(static_cast<Eigen::Index>(rows.size()));
    for (std::size_t k = 0; k < rows.size(); ++k) {
        rhs[k] = rowProduct(rows[k], projected.velocities) + rows[k].velocity_bias;
    }
    applyCorrection(rows, inverse_mass, solveMultipliers(rows, inverse_mass, rhs), -1.0, projected.velocities);
    validateState(model, projected);
    state = std::move(projected);
}
} // namespace physics
