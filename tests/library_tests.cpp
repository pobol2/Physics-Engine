// Tests of the reusable engine features: motions, the force library, custom
// forces and constraints, coupled constraint solving, adaptive integration, and
// diagnostics. Physical example systems are validated in example_tests.cpp.
#include "examples.hpp"
#include "physics/simulation.hpp"
#include "test_support.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {
using namespace physics;
using test::check;
using test::near;
using test::pi;
using test::rejects;
using test::vectorNear;

State restState(std::size_t count) {
    return {Vectors(count, Eigen::Vector3d::Zero()), Vectors(count, Eigen::Vector3d::Zero())};
}

void motions() {
    const std::vector<Motion> cases{
        Motion(1.0, 2.0, 3.0),
        Motion::linear({1.0, 0.0, -1.0}, {0.5, -2.0, 0.25}),
        Motion::oscillating({0.0, 1.0, 0.0}, {0.3, -0.2, 0.1}, 2.5, 0.4),
        Motion::circular({1.0, -1.0, 0.5}, {0.0, 0.7, 0.2}, {0.3, 0.0, 1.9}),
        Motion::custom([](double t) { return Eigen::Vector3d{t * t * t, std::sin(t), 1.0}; },
                       [](double t) { return Eigen::Vector3d{3 * t * t, std::cos(t), 0.0}; },
                       [](double t) { return Eigen::Vector3d{6 * t, -std::sin(t), 0.0}; })};
    for (const auto& motion : cases) {
        motion.validate();
        for (double t : {0.0, 0.37, 2.1}) {
            const double h = 1e-5;
            vectorNear((motion.position(t + h) - motion.position(t - h)) / (2 * h), motion.velocity(t), 1e-8,
                       "Motion velocity is the derivative of position");
            vectorNear((motion.velocity(t + h) - motion.velocity(t - h)) / (2 * h), motion.acceleration(t), 1e-8,
                       "Motion acceleration is the derivative of velocity");
        }
    }
    const auto circle = Motion::circular({1.0, -1.0, 0.5}, {0.0, 0.7, 0.2}, {0.0, 0.0, 2.0});
    near((circle.position(1.3) - Eigen::Vector3d{1.0, -1.0, 0.5}).norm(), std::hypot(0.7, 0.2), 1e-15,
         "Circular motion keeps its radius");
    vectorNear(circle.position(pi), circle.position(0.0), 1e-14, "Circular motion has period 2 pi / rate");
    vectorNear(Motion::circular(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), {0.0, 0.0, pi / 2}).position(1.0),
               Eigen::Vector3d::UnitY(), 1e-15, "Circular motion is right-handed about its axis");
    check(Motion().isStationary() && Motion().position(4.0).isZero(0.0), "Default motion is the origin");
    const Motion converted = Eigen::Vector3d::UnitX() * 2.0;
    vectorNear(converted.position(9.0), {2.0, 0.0, 0.0}, 0.0, "Vector expressions convert to stationary motion");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { Motion(nan, 0.0, 0.0).validate(); }, "Reject non-finite stationary point");
    rejects([&] { Motion::oscillating(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), nan).validate(); },
            "Reject non-finite frequency");
    rejects([] { Motion::custom({}, {}, {}).validate(); }, "Reject custom motion without functions");
    const auto bad = Motion::custom([](double) { return Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()); },
                                    [](double) { return Eigen::Vector3d::Zero(); },
                                    [](double) { return Eigen::Vector3d::Zero(); });
    rejects([&] { (void)bad.position(0.0); }, "Reject non-finite custom motion values");
}

// Force = -grad U for every conservative interaction, and dU/dt matches the
// reported potential rate (external power at rest without other work).
void checkConservative(const Model& model, const State& state, double time, const std::string& name) {
    const auto forces = evaluateForces(model, state, time);
    double scale = 1.0;
    for (const auto& f : forces) scale = std::max(scale, f.norm());
    for (std::size_t i = 0; i < state.positions.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            State plus = state, minus = state;
            const double h = 1e-6;
            plus.positions[i][axis] += h;
            minus.positions[i][axis] -= h;
            const double gradient = (energy(model, plus, time).potential - energy(model, minus, time).potential) / (2 * h);
            near(gradient, -forces[i][axis], 2e-7 * scale, name + ": force equals negative potential gradient");
        }
    }
    const double h = 1e-6;
    const double rate = (energy(model, state, time + h).potential - energy(model, state, time - h).potential) / (2 * h);
    near(evaluateDynamics(model, state, time).external_power, rate, 2e-7 * scale,
         name + ": potential rate equals partial dU/dt");
}

void forceLibrary() {
    State state = restState(3);
    state.positions = {{0.3, -0.4, 0.5}, {1.2, 0.1, -0.3}, {-0.6, 0.9, 0.2}};
    Model model;
    model.masses = {1.5, 2.0, 0.7};
    model.forces = {centralForce(0, Motion::oscillating({0.1, 0.0, 0.0}, {0.0, 0.2, 0.1}, 3.0), inverseSquare(2.0))};
    checkConservative(model, state, 0.4, "Central force about a moving center");
    model.forces = {pairForce(0, 1, lennardJones(0.8, 0.9)), pairForce(1, 2, inverseSquare(-1.5))};
    checkConservative(model, state, 0.0, "Pair forces");
    model.forces = {gravitation(1.3, {}, 0.05)};
    checkConservative(model, state, 0.0, "Softened gravitation");
    Eigen::Vector3d total = Eigen::Vector3d::Zero();
    for (const auto& f : evaluateForces(model, state, 0.0)) total += f;
    vectorNear(total, Eigen::Vector3d::Zero(), 1e-15, "Gravitation obeys Newton's third law");
    model.forces = {gravitation(1.0, {0, 2})};
    const Eigen::Vector3d d = state.positions[2] - state.positions[0];
    vectorNear(evaluateForces(model, state, 0.0)[0], d * (1.5 * 0.7 / std::pow(d.norm(), 3)), 1e-14,
               "Gravitation on a particle subset");
    check(evaluateForces(model, state, 0.0)[1].isZero(0.0), "Unlisted particles feel no gravitation");

    CustomForce quartic;
    quartic.particles = {1};
    quartic.apply = [](const SystemView& s, Vectors& f) {
        f[1] -= s.state.positions[1] * (2.0 * (1.0 + s.time) * s.state.positions[1].squaredNorm());
    };
    quartic.potential = [](const SystemView& s) {
        return 0.5 * (1.0 + s.time) * std::pow(s.state.positions[1].squaredNorm(), 2);
    };
    quartic.potential_rate = [](const SystemView& s) { return 0.5 * std::pow(s.state.positions[1].squaredNorm(), 2); };
    model.forces = {customForce(quartic)};
    checkConservative(model, state, 0.8, "Custom time-dependent potential");

    // Nonconservative forces.
    state.velocities = {{3.0, 4.0, 0.0}, {0.0, -1.0, 2.0}, {1.0, 1.0, 1.0}};
    model.forces = {quadraticDrag(0, 0.5)};
    vectorNear(evaluateForces(model, state, 0.0)[0], {-7.5, -10.0, 0.0}, 1e-14, "Quadratic drag is -c|v|v");
    model.forces = {harmonicDrive(2, {0.0, 3.0, 0.0}, 2.0, 0.5)};
    vectorNear(evaluateForces(model, state, 1.0)[2], {0.0, 3.0 * std::cos(2.5), 0.0}, 1e-15, "Harmonic drive value");
    model.forces = {lorentzForce(1, -2.0, uniformField({1.0, 0.0, 0.0}), uniformField({0.0, 0.0, 3.0}))};
    vectorNear(evaluateForces(model, state, 0.0)[1], -2.0 * Eigen::Vector3d{1.0 - 3.0, 0.0, 0.0}, 1e-15,
               "Lorentz force q (E + v x B)");
    model.forces = {lorentzForce(1, 2.0, {}, [](const Eigen::Vector3d& q, double t) {
        return Eigen::Vector3d{q.y(), t, 1.0};
    })};
    near(evaluateDynamics(model, state, 0.3).external_power, 0.0, 1e-14, "Magnetic force does no work");
    near(energy(model, state, 0.0).potential, 0.0, 0.0, "Nonconservative forces have no potential");

    // Validation of the library.
    const auto invalid = [&](ForcePtr force, const char* message) {
        Model broken;
        broken.masses = {1.0, 1.0};
        broken.forces = {std::move(force)};
        rejects([&] { validateModel(broken); }, message);
    };
    invalid(nullptr, "Reject null force");
    invalid(drivingForce(2, [](double) { return Eigen::Vector3d::Zero(); }), "Reject drive index");
    invalid(drivingForce(0, {}), "Reject drive without function");
    invalid(pairForce(1, 1, inverseSquare(1.0)), "Reject self pair");
    invalid(gravitation(0.0), "Reject zero gravitational constant");
    invalid(gravitation(1.0, {0, 0}), "Reject duplicate gravitation particle");
    invalid(quadraticDrag(0, -1.0), "Reject negative drag");
    invalid(lorentzForce(0, 1.0, {}, {}), "Reject Lorentz force without fields");
    invalid(customForce({}), "Reject custom force without apply");
    CustomForce rate_only;
    rate_only.apply = [](const SystemView&, Vectors&) {};
    rate_only.potential_rate = [](const SystemView&) { return 0.0; };
    invalid(customForce(rate_only), "Reject potential rate without potential");
    rejects([] { (void)inverseSquare(std::numeric_limits<double>::infinity()); }, "Reject infinite strength");
    rejects([] { (void)lennardJones(1.0, 0.0); }, "Reject zero Lennard-Jones radius");
    Model coincident;
    coincident.masses = {1.0, 1.0};
    coincident.forces = {pairForce(0, 1, inverseSquare(1.0))};
    rejects([&] { evaluateForces(coincident, restState(2), 0.0); }, "Reject coincident radial pair");
}

// One system using every feature, checked against the identity
// external_power = dE/dt along the equations of motion.
Model kitchenSink() {
    Model model;
    model.masses = {1.0, 0.8, 1.3, 0.9, 1.1};
    model.gravity = {0.0, -9.81, 0.0};
    model.prescribed = {{0, Motion::circular({0.0, 2.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 1.7})}};
    model.links = {{0, 1, 1.0}};
    model.distance_constraints = {{2, Motion::oscillating({0.0, -2.0, 0.0}, {0.0, 0.1, 0.0}, 3.0), 1.0}};
    model.constraints = {planeConstraint(2, {0.0, -2.0, 0.0}, Eigen::Vector3d::UnitZ(), {0.0, 1.2, 0.0})};
    model.harmonic_forces = {{3, Motion::linear({2.0, 0.0, 0.0}, {0.0, 0.3, 0.0}), 5.0}};
    model.springs = {{3, 4, 2.0, 0.5}, {1, 3, 1.5, 0.0}};
    model.damping = {{3, 0.2}};
    CustomForce modulated;
    modulated.particles = {4};
    modulated.apply = [](const SystemView& s, Vectors& f) {
        f[4] -= s.state.positions[4] * (1.0 + 0.5 * std::sin(2.0 * s.time));
    };
    modulated.potential = [](const SystemView& s) {
        return 0.5 * (1.0 + 0.5 * std::sin(2.0 * s.time)) * s.state.positions[4].squaredNorm();
    };
    modulated.potential_rate = [](const SystemView& s) {
        return 0.5 * std::cos(2.0 * s.time) * s.state.positions[4].squaredNorm();
    };
    model.forces = {harmonicDrive(3, {1.0, 0.0, 0.5}, 1.3), quadraticDrag(4, 0.3),
                    centralForce(4, Motion::linear({3.0, 1.0, 0.0}, {0.2, 0.0, 0.0}), inverseSquare(1.0)),
                    lorentzForce(1, 0.5, uniformField({0.0, 0.2, 0.0}), uniformField({0.0, 0.0, 1.0})),
                    customForce(modulated)};
    return model;
}

State kitchenSinkState(const Model& model, double time) {
    State state{{Eigen::Vector3d::Zero(), {0.2, 1.1, 0.3}, {0.8, -2.6, 0.0}, {2.4, 0.3, -0.2}, {1.5, 0.8, 0.4}},
                {Eigen::Vector3d::Zero(), {0.4, -0.3, 0.2}, {0.5, 0.6, 0.0}, {-0.3, 0.2, 0.1}, {0.6, -0.4, 0.3}}};
    projectConstraints(model, state, time);
    return state;
}

void externalPower() {
    const Model model = kitchenSink();
    const double t = 0.7;
    const State state = kitchenSinkState(model, t);
    const auto dynamics = evaluateDynamics(model, state, t);
    const double h = 1e-5;
    State plus = state, minus = state;
    for (std::size_t i = 0; i < state.positions.size(); ++i) {
        plus.positions[i] += h * state.velocities[i];
        plus.velocities[i] += h * dynamics.accelerations[i];
        minus.positions[i] -= h * state.velocities[i];
        minus.velocities[i] -= h * dynamics.accelerations[i];
    }
    const double rate = (energy(model, plus, t + h).total() - energy(model, minus, t - h).total()) / (2 * h);
    near(dynamics.external_power, rate, 1e-6 * std::max(1.0, std::abs(rate)), "External power is dE/dt");
    check(std::abs(dynamics.external_power) > 0.1, "Kitchen-sink system exchanges energy with its surroundings");
    const auto error = constraintError(model, state, t);
    near(error.max_position_error, 0.0, 1e-14, "Projection satisfies coupled and moving constraints");
    near(error.max_velocity_error, 0.0, 1e-14, "Projection satisfies velocity constraints");

    // The integrated work balances the energy change.
    Simulation sim(model, state, t);
    sim.advanceTo(t + 2.0, 0.0005);
    auto d = sim.diagnostics();
    check(std::abs(d.external_work) > 0.1, "Kitchen-sink system does measurable external work");
    near(d.energy_balance_error, 0.0, 1e-7, "RK4 energy balance with every force and constraint type");
    near(d.constraints.max_position_error, 0.0, 1e-13, "RK4 keeps coupled constraints");
    Simulation adaptive(model, state, t);
    adaptive.advanceAdaptive(t + 2.0, {1e-10, 1e-10});
    d = adaptive.diagnostics();
    near(d.energy_balance_error, 0.0, 1e-6, "Adaptive energy balance with every force and constraint type");
    near(d.constraints.max_velocity_error, 0.0, 1e-13, "Adaptive integration keeps velocity constraints");
    vectorNear(adaptive.state().positions[3], sim.state().positions[3], 1e-6, "Adaptive and RK4 agree");
    vectorNear(adaptive.state().positions[0], model.prescribed[0].motion.position(t + 2.0), 0.0,
               "Prescribed particle follows its motion exactly");
}

void coupledConstraints() {
    // Projection onto two coupled rods from a state off both of them.
    auto example = examples::doublePendulum();
    example.initial_state.positions = {{0.9, -0.6, 0.1}, {1.1, -1.9, -0.2}};
    example.initial_state.velocities = {{1.0, 0.5, 0.2}, {-0.3, 2.0, 0.4}};
    State projected = example.initial_state;
    projectConstraints(example.model, projected, 0.0);
    near(projected.positions[0].norm(), 1.0, 1e-15, "First rod length after coupled projection");
    near((projected.positions[1] - projected.positions[0]).norm(), 1.0, 1e-15, "Second rod length");
    near(constraintError(example.model, projected, 0.0).max_velocity_error, 0.0, 1e-15,
         "Coupled velocity projection");

    // Mass-weighted projection of a free link preserves the centre of mass and momentum.
    Model pair;
    pair.masses = {1.0, 3.0};
    pair.links = {{0, 1, 2.0}};
    State loose{{{0.0, 0.0, 0.0}, {3.0, 1.0, 0.0}}, {{1.0, 2.0, 0.0}, {-1.0, 0.5, 1.0}}};
    State tight = loose;
    projectConstraints(pair, tight, 0.0);
    const auto centre = [&](const State& s) { return (s.positions[0] * 1.0 + s.positions[1] * 3.0) / 4.0; };
    vectorNear(centre(tight), centre(loose), 1e-15, "Projection preserves the centre of mass");
    vectorNear(momentum(pair, tight).linear, momentum(pair, loose).linear, 1e-15, "Projection preserves momentum");

    // A freely spinning rigid triangle conserves energy and angular momentum.
    Model triangle;
    triangle.masses = {1.0, 2.0, 1.5};
    triangle.links = {{0, 1, 1.0}, {1, 2, 1.2}, {2, 0, 0.9}};
    const double x2 = (0.9 * 0.9 - 1.2 * 1.2 + 1.0) / 2.0;
    State spin{{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {x2, std::sqrt(0.81 - x2 * x2), 0.0}}, Vectors(3)};
    const Eigen::Vector3d omega{0.3, -0.2, 2.0};
    for (std::size_t i = 0; i < 3; ++i) spin.velocities[i] = omega.cross(spin.positions[i]) + Eigen::Vector3d{0.1, 0.0, 0.0};
    Simulation rigid(triangle, spin);
    const auto initial = rigid.diagnostics();
    rigid.advanceAdaptive(10.0, {1e-11, 1e-11});
    const auto final = rigid.diagnostics();
    near(final.energy_change, 0.0, 1e-8, "Rigid triangle energy");
    vectorNear(final.momentum.angular, initial.momentum.angular, 1e-8, "Rigid triangle angular momentum");
    vectorNear(final.momentum.linear, initial.momentum.linear, 1e-12, "Rigid triangle linear momentum");
    near(final.constraints.max_position_error, 0.0, 1e-14, "Rigid triangle shape");

    // A hanging chain of five links: one particle has two rows, all coupled.
    Model chain;
    chain.masses = {1.0, 1.0, 1.0, 1.0, 1.0};
    chain.gravity = {0.0, -9.81, 0.0};
    chain.distance_constraints = {{0, Eigen::Vector3d::Zero(), 0.5}};
    State hanging = restState(5);
    for (std::size_t i = 0; i < 5; ++i) {
        hanging.positions[i] = {0.5 * (i + 1.0) * std::sin(1.0), -0.5 * (i + 1.0) * std::cos(1.0), 0.0};
        if (i > 0) chain.links.push_back({i - 1, i, 0.5});
    }
    Simulation swinging(chain, hanging);
    swinging.advanceAdaptive(5.0, {1e-10, 1e-10});
    near(swinging.diagnostics().constraints.max_position_error, 0.0, 1e-13, "Chain links stay rigid");
    near(swinging.diagnostics().energy_change, 0.0, 1e-6, "Chain energy is conserved");

    // Inconsistent and singular constraint sets are rejected.
    Model impossible;
    impossible.masses = {1.0};
    impossible.distance_constraints = {{0, Eigen::Vector3d::Zero(), 1.0}};
    impossible.constraints = {planeConstraint(0, {2.0, 0.0, 0.0}, Eigen::Vector3d::UnitX())};
    State start{{{0.5, 0.5, 0.0}}, {Eigen::Vector3d::Zero()}};
    const State before = start;
    rejects([&] { projectConstraints(impossible, start, 0.0); }, "Reject a sphere and a disjoint plane");
    vectorNear(start.positions[0], before.positions[0], 0.0, "Failed projection leaves the state unchanged");
    Model invalid;
    invalid.masses = {1.0, 1.0};
    invalid.prescribed = {{0, Eigen::Vector3d::Zero()}, {1, Eigen::Vector3d::UnitX()}};
    invalid.links = {{0, 1, 1.0}};
    rejects([&] { validateModel(invalid); }, "Reject a link between two prescribed particles");
    invalid.links = {{0, 0, 1.0}};
    rejects([&] { validateModel(invalid); }, "Reject a self link");
    invalid.links.clear();
    invalid.prescribed.pop_back();
    invalid.constraints = {nullptr};
    rejects([&] { validateModel(invalid); }, "Reject null constraint");
    invalid.constraints = {planeConstraint(0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero())};
    rejects([&] { validateModel(invalid); }, "Reject zero plane normal");
}

void customConstraints() {
    // A particle on an inclined plane slides with acceleration g sin(alpha).
    const double alpha = 0.4;
    Model incline;
    incline.masses = {2.0};
    incline.gravity = {0.0, -9.81, 0.0};
    const Eigen::Vector3d normal{-std::sin(alpha), std::cos(alpha), 0.0};
    incline.constraints = {planeConstraint(0, Eigen::Vector3d::Zero(), normal)};
    State rest = restState(1);
    vectorNear(evaluateAccelerations(incline, rest, 0.0)[0],
               Eigen::Vector3d{std::cos(alpha), std::sin(alpha), 0.0} * (-9.81 * std::sin(alpha)), 1e-14,
               "Inclined-plane acceleration");

    // A user-defined spherical constraint reproduces the built-in distance constraint.
    CustomConstraint sphere;
    sphere.particles = {0};
    sphere.rows = [](const SystemView& s, std::vector<ConstraintRow>& rows) {
        const Eigen::Vector3d& q = s.state.positions[0];
        const Eigen::Vector3d& v = s.state.velocities[0];
        ConstraintRow row;
        row.terms = {{0, q / q.norm()}};
        row.value = q.norm() - 1.3;
        row.acceleration_bias = v.squaredNorm() / q.norm();
        row.scale = 1.3;
        rows.push_back(row);
    };
    auto builtin = examples::pendulum({2.0, 1.3, 9.81, 1.0, 0.5});
    Model custom = builtin.model;
    custom.distance_constraints.clear();
    custom.constraints = {customConstraint(sphere)};
    Simulation a(builtin.model, builtin.initial_state), b(custom, builtin.initial_state);
    a.advanceTo(3.0, 0.001);
    b.advanceTo(3.0, 0.001);
    vectorNear(b.state().positions[0], a.state().positions[0], 1e-13, "Custom constraint matches built-in rod");
    CustomConstraint undeclared = sphere;
    undeclared.particles = {};
    custom.constraints = {customConstraint(undeclared)};
    rejects([&] { Simulation(custom, builtin.initial_state); }, "Reject rows on undeclared particles");
    rejects([] { customConstraint({})->validate(1); }, "Reject custom constraint without rows");
}

double oscillatorError(const AdaptiveSettings& settings, std::uint64_t* steps = nullptr) {
    Simulation sim(examples::oscillator({2.0, 8.0, 0.7, -0.3}).model, examples::oscillator({2.0, 8.0, 0.7, -0.3}).initial_state);
    sim.advanceAdaptive(10.0, settings);
    check(sim.time() == 10.0, "Adaptive integration lands exactly on the end time");
    if (steps) *steps = sim.steps();
    return std::hypot(sim.state().positions[0].x() - (0.7 * std::cos(20.0) - 0.15 * std::sin(20.0)),
                      (sim.state().velocities[0].x() - (-1.4 * std::sin(20.0) - 0.3 * std::cos(20.0))) / 2.0);
}

void adaptiveIntegration() {
    double previous = 1.0;
    std::uint64_t previous_steps = 0;
    for (double tolerance : {1e-6, 1e-8, 1e-10, 1e-12}) {
        std::uint64_t steps = 0;
        const double error = oscillatorError({tolerance, tolerance}, &steps);
        check(error < previous / 10.0, "Adaptive error decreases with the tolerance");
        check(error < 2000.0 * tolerance, "Adaptive global error is proportional to the tolerance");
        check(steps > previous_steps, "Tighter tolerance takes more steps");
        previous = error;
        previous_steps = steps;
    }

    auto example = examples::oscillator();
    Simulation sim(example.model, example.initial_state);
    sim.advanceAdaptive(5.0);
    const auto& s = sim.statistics();
    check(s.evaluations == 7 * (s.accepted + s.rejected) + 2, "Evaluation count: 7 per attempt plus 2 to start");
    check(s.min_step > 0.0 && s.min_step <= s.max_step && s.last_step > 0.0, "Step statistics are consistent");
    // Stopping at output times keeps the step-size controller's state.
    Simulation gridded(example.model, example.initial_state);
    for (int k = 1; k <= 50; ++k) gridded.advanceAdaptive(0.1 * k);
    check(gridded.steps() < sim.steps() + 60, "Output stops add at most one step each");
    vectorNear(gridded.state().positions[0], sim.state().positions[0], 1e-8, "Output stops do not change accuracy");
    const auto steps = sim.steps();
    sim.advanceAdaptive(sim.time());
    check(sim.steps() == steps, "Advancing to the current time does no work");

    // A step that passes through a singular configuration is rejected and retried.
    Model spring;
    spring.masses = {1.0, 1.0};
    spring.springs = {{0, 1, 4.0, 1.0}};
    State collision{{Eigen::Vector3d::Zero(), {2.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}}};
    Simulation bounce(spring, collision);
    AdaptiveSettings large{1e-11, 1e-11};
    large.initial_step = 1.0;
    bounce.advanceAdaptive(3.0, large);
    check(bounce.statistics().rejected > 0, "Singular trial step was rejected");
    near(bounce.diagnostics().energy_change, 0.0, 1e-7, "Recovered bounce conserves energy");

    // A force that becomes undefined makes the integrator stop with an error.
    Model failing;
    failing.masses = {1.0};
    CustomForce cliff;
    cliff.apply = [](const SystemView& system, Vectors&) {
        if (system.time > 0.5) throw std::domain_error("force undefined beyond t = 0.5");
    };
    failing.forces = {customForce(cliff)};
    Simulation stuck(failing, restState(1));
    rejects([&] { stuck.advanceAdaptive(1.0); }, "Report failure when the step size underflows");
    check(stuck.time() <= 0.5, "No step is accepted past the failure");
    AdaptiveSettings limited;
    limited.max_steps = 3;
    Simulation capped(example.model, example.initial_state);
    rejects([&] { capped.advanceAdaptive(100.0, limited); }, "Enforce max_steps");

    // Systems without free coordinates still advance.
    Model driven;
    driven.masses = {1.0};
    driven.prescribed = {{0, Motion::oscillating(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 2.0)}};
    Simulation kinematic(driven, restState(1));
    kinematic.advanceAdaptive(3.0);
    vectorNear(kinematic.state().positions[0], {std::cos(6.0), 0.0, 0.0}, 0.0, "Fully prescribed system");

    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const AdaptiveSettings& bad : {AdaptiveSettings{0.0, 0.0}, AdaptiveSettings{nan, 1e-9},
                                        AdaptiveSettings{1e-9, -1.0}, AdaptiveSettings{1e-9, 1e-9, -1.0},
                                        AdaptiveSettings{1e-9, 1e-9, 0.0, 1e-3, 1e-2}}) {
        rejects([&] { sim.advanceAdaptive(6.0, bad); }, "Reject invalid adaptive settings");
    }
    rejects([&] { sim.advanceAdaptive(1.0); }, "Reject backwards adaptive time");
}

void diagnostics() {
    // Damping: the energy loss is booked as external work, not numerical error.
    auto damped = examples::oscillator({2.0, 8.0, 1.0, 0.2, 1.0});
    Simulation sim(damped.model, damped.initial_state);
    sim.advanceTo(5.0, 0.001);
    auto d = sim.diagnostics();
    check(d.energy_change < -1.0, "Damped oscillator loses energy");
    near(d.external_work, d.energy_change, 1e-11, "Damping work equals the energy change");
    near(d.energy_balance_error, 0.0, 1e-11, "Damped energy balance");
    near(d.scaled_energy_balance_error, d.energy_balance_error / 4.04, 1e-15,
         "Scaled balance uses the initial energy");

    Model model;
    model.masses = {2.0, 3.0};
    State state{{{1.0, 0.0, 0.0}, {0.0, 2.0, 0.0}}, {{0.0, 1.0, 0.0}, {1.0, 0.0, 1.0}}};
    const auto p = momentum(model, state);
    vectorNear(p.linear, {3.0, 2.0, 3.0}, 0.0, "Linear momentum");
    vectorNear(p.angular, Eigen::Vector3d{0.0, 0.0, 2.0} + Eigen::Vector3d{6.0, 0.0, -6.0}, 0.0,
               "Angular momentum about the origin");

    Simulation fresh(model, state, 2.5);
    near(fresh.time(), 2.5, 0.0, "Simulation starts at the requested time");
    check(fresh.diagnostics().steps.accepted == 0 && fresh.diagnostics().external_work == 0.0,
          "Fresh simulation has no steps or work");
    rejects([&] { Simulation(model, state, std::numeric_limits<double>::infinity()); }, "Reject infinite start time");
}

void exampleCatalog() {
    check(examples::catalog().size() == 10, "Catalog lists every example");
    for (const auto& spec : examples::catalog()) {
        auto example = examples::build(spec);
        Simulation fixed(example.model, example.initial_state);
        fixed.advanceTo(0.2, spec.dt);
        Simulation adaptive(example.model, example.initial_state);
        adaptive.advanceAdaptive(0.2);
        for (const auto* sim : {&fixed, &adaptive}) {
            const auto d = sim->diagnostics();
            // Default tolerances; the Kapitza drive's work integral is the least accurate.
            near(d.scaled_energy_balance_error, 0.0, 1e-6, spec.name + " energy balance");
            near(d.constraints.max_position_error, 0.0, 1e-12, spec.name + " constraints");
        }
        check(spec.duration > 0.0 && spec.dt > 0.0 && !spec.summary.empty(), spec.name + " has run defaults");
    }
    rejects([] { (void)examples::find("missing"); }, "Reject unknown example");
    rejects([] { (void)examples::build(examples::find("kepler"), {{"angle", 1.0}}); }, "Reject unknown parameter");
    const auto custom = examples::build(examples::find("pendulum"), {{"length", 2.0}});
    near(custom.model.distance_constraints[0].length, 2.0, 0.0, "Parameter overrides apply");
    rejects([] { (void)examples::kepler({1.0, 1.0, 1.0, 1.0}); }, "Reject parabolic Kepler orbit");
    rejects([] { (void)examples::parametricOscillator({1.0, 1.0, 1.0}); }, "Reject |h| >= 1");
}
} // namespace

int main(int argc, char** argv) {
    return test::run(argc, argv, {
        {"motions", motions}, {"force_library", forceLibrary}, {"external_power", externalPower},
        {"coupled_constraints", coupledConstraints}, {"custom_constraints", customConstraints},
        {"adaptive_integration", adaptiveIntegration}, {"diagnostics", diagnostics},
        {"example_catalog", exampleCatalog}});
}
