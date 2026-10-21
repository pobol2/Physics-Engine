#include "examples.hpp"
#include "physics/simulation.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
using namespace physics;
using test::check;
using test::near;
using test::pi;
using test::rejects;
using test::vectorNear;

Simulation simulation(examples::Example example) {
    return Simulation(std::move(example.model), std::move(example.initial_state));
}

void forcesEnergy() {
    Model model;
    model.masses = {2.0, 3.0};
    model.gravity = {0.0, -9.81, 0.0};
    model.harmonic_forces = {{0, {1.0, 2.0, 0.0}, 7.0}};
    State state{{{1.4, 2.6, 0.1}, {-0.3, 4.0, 0.2}}, {{2.0, 3.0, 1.0}, {1.0, 0.0, 0.0}}};
    validateModel(model);
    const auto forces = evaluateForces(model, state, 0.5);
    vectorNear(forces[0], {-2.8, -23.82, -0.7}, 1e-13, "Hooke and gravity force");
    near(energy(model, state, 0.0).kinetic, 15.5, 1e-13, "Kinetic energy includes masses");
    const double expected_potential = 2.0 * 9.81 * 2.6 + 3.0 * 9.81 * 4.0
        + 0.5 * 7.0 * (0.4 * 0.4 + 0.6 * 0.6 + 0.1 * 0.1);
    near(energy(model, state, 0.0).potential, expected_potential, 1e-12, "Potential energy");
    for (std::size_t i = 0; i < state.positions.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            auto plus = state, minus = state;
            plus.positions[i][axis] += 1e-5;
            minus.positions[i][axis] -= 1e-5;
            const double gradient = (energy(model, plus, 0.0).potential - energy(model, minus, 0.0).potential) / 2e-5;
            near(gradient, -forces[i][axis], 3e-9, "Force equals negative energy gradient");
        }
    }
    auto stage = state;
    stage.positions[0].x() += 0.2;
    near(evaluateForces(model, stage, 0.75)[0].x(), -4.2, 1e-13, "Evaluate supplied intermediate state");
    vectorNear(evaluateForces(model, state, 1.0)[0], forces[0], 0.0, "No force accumulation");
    near(state.positions[0].x(), 1.4, 0.0, "Evaluation leaves input state unchanged");
}

void initialization() {
    Model free_model;
    free_model.masses = {1.0, 2.0};
    check(free_model.gravity.isZero(0.0), "Default gravity is explicitly zero");
    check(HarmonicForce{}.equilibrium.position(0.0).isZero(0.0), "Default equilibrium is explicitly zero");
    check(DistanceConstraint{}.pivot.position(0.0).isZero(0.0), "Default pivot is explicitly zero");
    check(PrescribedMotion{}.motion.position(0.0).isZero(0.0), "Default prescribed motion is explicitly zero");
    State free_state{{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()},
                     {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
    for (const Eigen::Vector3d& force : evaluateForces(free_model, free_state, 0.0)) {
        check(force.allFinite() && force.isZero(0.0), "Force buffers start at zero");
    }
    auto example = examples::pendulum();
    example.initial_state.positions[0] = {2.0, 0.0, 0.0};
    example.initial_state.velocities[0] = {3.0, 4.0, 0.0};
    auto sim = simulation(example);
    vectorNear(sim.state().positions[0], {1.0, 0.0, 0.0}, 1e-15, "Initial length projection");
    vectorNear(sim.state().velocities[0], {0.0, 4.0, 0.0}, 1e-15, "Initial tangent projection");
    near(sim.diagnostics().energy_change, 0.0, 0.0, "Energy baseline after projection");
    const auto acceleration = evaluateAccelerations(sim.model(), sim.state(), 0.0)[0];
    const auto velocity = sim.state().velocities[0];
    near(sim.state().positions[0].dot(acceleration) + velocity.dot(velocity), 0.0, 1e-13,
         "Acceleration satisfies differentiated constraint");
    const Eigen::Vector3d reaction = acceleration * sim.model().masses[0] - evaluateForces(sim.model(), sim.state(), 0.0)[0];
    near(velocity.dot(reaction), 0.0, 1e-13, "Ideal reaction does no work");
    example.model.masses[0] = 100.0;
    near(sim.model().masses[0], 1.0, 0.0, "Simulation owns its configuration");
}

double oscillatorError(double dt) {
    examples::OscillatorConfig config{2.0, 8.0, 0.7, -0.3};
    auto sim = simulation(examples::oscillator(config));
    sim.advanceTo(1.0, dt);
    const double omega = std::sqrt(config.stiffness / config.mass);
    const double position = config.displacement * std::cos(omega) + config.velocity / omega * std::sin(omega);
    const double velocity = -config.displacement * omega * std::sin(omega) + config.velocity * std::cos(omega);
    return std::hypot(sim.state().positions[0].x() - position, (sim.state().velocities[0].x() - velocity) / omega);
}

void oscillatorAccuracy() {
    const examples::OscillatorConfig config{2.0, 8.0, 0.7, -0.3};
    auto sim = simulation(examples::oscillator(config));
    double max_error = 0.0;
    for (int step = 0; step < 1000; ++step) {
        sim.step(0.01);
        const double t = sim.time();
        const double x = config.displacement * std::cos(2.0 * t) + config.velocity / 2.0 * std::sin(2.0 * t);
        const double v = -2.0 * config.displacement * std::sin(2.0 * t) + config.velocity * std::cos(2.0 * t);
        max_error = std::max(max_error, std::hypot(sim.state().positions[0].x() - x,
                                                 (sim.state().velocities[0].x() - v) / 2.0));
    }
    near(max_error, 0.0, 2e-8, "Oscillator analytical solution over ten seconds");
    std::cout << "max analytical phase-space error=" << max_error << '\n';
}

void oscillatorConvergence() {
    const double coarse = oscillatorError(0.1), medium = oscillatorError(0.05), fine = oscillatorError(0.025);
    check(coarse / medium > 14.0 && coarse / medium < 18.0, "Oscillator must converge at fourth order");
    check(medium / fine > 14.0 && medium / fine < 18.0, "Oscillator refinement must keep fourth order");
    std::cout << "error ratios=" << coarse / medium << ", " << medium / fine << '\n';
}

void pendulumSmallAngle() {
    examples::PendulumConfig config{2.5, 1.3, 9.81, 1e-4, 0.0};
    auto sim = simulation(examples::pendulum(config));
    sim.advanceTo(5.0, 0.002);
    const auto p = sim.state().positions[0];
    const double angle = std::atan2(p.x(), -p.y());
    near(angle, config.angle * std::cos(std::sqrt(config.gravity / config.length) * sim.time()),
         2e-10, "Small-angle pendulum solution");
}

// Independent generalized-coordinate RK4 reference: theta'' = -(g/l) sin(theta).
std::pair<double, double> angularReference(double time, const examples::PendulumConfig& config) {
    double angle = config.angle, omega = config.angular_velocity;
    const int count = static_cast<int>(std::ceil(time / 0.0001));
    const double h = time / count;
    const auto acceleration = [&](double theta, double velocity) {
        return -config.gravity / config.length * std::sin(theta) - config.damping / config.mass * velocity;
    };
    for (int i = 0; i < count; ++i) {
        const double a1 = omega, b1 = acceleration(angle, omega);
        const double a2 = omega + h * b1 / 2, b2 = acceleration(angle + h * a1 / 2, a2);
        const double a3 = omega + h * b2 / 2, b3 = acceleration(angle + h * a2 / 2, a3);
        const double a4 = omega + h * b3, b4 = acceleration(angle + h * a3, a4);
        angle += h / 6 * (a1 + 2 * a2 + 2 * a3 + a4);
        omega += h / 6 * (b1 + 2 * b2 + 2 * b3 + b4);
    }
    return {angle, omega};
}

void pendulumNonlinear() {
    const examples::PendulumConfig config{2.5, 1.3, 9.81, 1.2, 0.3};
    const auto reference = angularReference(1.7, config);
    const auto error = [&](double dt) {
        auto sim = simulation(examples::pendulum(config));
        sim.advanceTo(1.7, dt);
        const auto p = sim.state().positions[0], v = sim.state().velocities[0];
        const double angle = std::atan2(p.x(), -p.y());
        const double omega = (p.x() * v.y() - p.y() * v.x()) / (config.length * config.length);
        return std::hypot(angle - reference.first, (omega - reference.second) * std::sqrt(config.length / config.gravity));
    };
    const double coarse = error(0.02), medium = error(0.01), fine = error(0.005);
    near(fine, 0.0, 2e-8, "Nonlinear pendulum reference");
    check(coarse / medium > 12.0 && coarse / medium < 20.0, "Constrained pendulum fourth-order convergence");
    check(medium / fine > 12.0 && medium / fine < 20.0, "Constrained pendulum refined convergence");
    std::cout << "nonlinear error=" << fine << ", ratios=" << coarse / medium << ", " << medium / fine << '\n';
}

void pendulumPeriod() {
    const examples::PendulumConfig config{1.0, 1.0, 9.81, 1.1, 0.0};
    // Complete elliptic integral via independent Simpson quadrature.
    const double modulus = std::sin(config.angle / 2.0);
    const int intervals = 2000;
    const double h = (pi / 2.0) / intervals;
    double integral = 0.0;
    for (int i = 0; i <= intervals; ++i) {
        const double sine = std::sin(i * h);
        const double value = 1.0 / std::sqrt(1.0 - modulus * modulus * sine * sine);
        integral += (i == 0 || i == intervals ? 1 : (i % 2 == 0 ? 2 : 4)) * value;
    }
    const double exact_period = 4.0 * std::sqrt(config.length / config.gravity) * integral * h / 3.0;
    auto sim = simulation(examples::pendulum(config));
    double previous_x = sim.state().positions[0].x();
    double measured_period = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const double previous_time = sim.time();
        sim.step(0.001);
        const double x = sim.state().positions[0].x();
        if (x <= 0.0) {
            measured_period = 4.0 * (previous_time + 0.001 * previous_x / (previous_x - x));
            break;
        }
        previous_x = x;
    }
    near(measured_period, exact_period, 2e-7, "Finite-amplitude pendulum period");
    check(exact_period > 1.07 * 2.0 * pi * std::sqrt(config.length / config.gravity),
          "Finite-amplitude example must differ from small-angle approximation");
    std::cout << "measured period=" << measured_period << ", elliptic period=" << exact_period << '\n';
}

void conservation() {
    auto oscillator = simulation(examples::oscillator({2.0, 8.0, 1.0, 0.2}));
    auto pendulum = simulation(examples::pendulum({2.0, 1.2, 9.81, 1.1, 0.0}));
    double oscillator_drift = 0.0, pendulum_drift = 0.0, length_error = 0.0, radial_velocity = 0.0;
    for (int i = 0; i < 10000; ++i) {
        oscillator.step(0.005);
        pendulum.step(0.005);
        const auto od = oscillator.diagnostics(), pd = pendulum.diagnostics();
        oscillator_drift = std::max(oscillator_drift, std::abs(od.energy_change));
        pendulum_drift = std::max(pendulum_drift, std::abs(pd.energy_change));
        length_error = std::max(length_error, pd.constraints.max_position_error);
        radial_velocity = std::max(radial_velocity, pd.constraints.max_velocity_error);
    }
    near(oscillator_drift, 0.0, 1e-8, "Oscillator energy drift over 50 seconds");
    near(pendulum_drift, 0.0, 2e-6, "Pendulum energy drift over 50 seconds");
    near(length_error, 0.0, 1e-14, "Pendulum length over 50 seconds");
    near(radial_velocity, 0.0, 1e-13, "Pendulum tangent velocity over 50 seconds");
    std::cout << "max energy drift (J): oscillator=" << oscillator_drift << ", pendulum=" << pendulum_drift
              << "; length error=" << length_error << "; radial velocity=" << radial_velocity << '\n';
}

void validation() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    rejects([] { (void)examples::oscillator({0.0, 4.0, 1.0, 0.0}); }, "Reject mass zero");
    rejects([] { (void)examples::oscillator({1.0, -4.0, 1.0, 0.0}); }, "Reject negative stiffness");
    rejects([&] { (void)examples::oscillator({1.0, 4.0, nan, 0.0}); }, "Reject nonfinite state");
    rejects([] { (void)examples::pendulum({1.0, 0.0, 9.81, 0.2, 0.0}); }, "Reject zero length");
    rejects([] { (void)examples::pendulum({1.0, 1.0, -9.81, 0.2, 0.0}); }, "Reject invalid gravity");
    auto example = examples::pendulum();
    example.model.distance_constraints.push_back(example.model.distance_constraints[0]);
    (void)simulation(example); // A consistent redundant constraint is solved by minimum norm.
    example.model.distance_constraints[1].length = 2.0;
    rejects([&] { (void)simulation(example); }, "Reject conflicting constraints on one particle");
    example = examples::pendulum();
    example.initial_state.positions[0] = Eigen::Vector3d::Zero();
    rejects([&] { (void)simulation(example); }, "Reject singular pivot initialization");
    example = examples::oscillator();
    example.initial_state.velocities.clear();
    rejects([&] { (void)simulation(example); }, "Reject mismatched state shape");
    example = examples::oscillator();
    example.model.harmonic_forces[0].particle = 2;
    rejects([&] { (void)simulation(example); }, "Reject invalid particle index");
    auto sim = simulation(examples::oscillator());
    for (double dt : {0.0, -0.1, nan, infinity}) {
        rejects([&] { sim.step(dt); }, "Reject invalid step");
    }
    rejects([&] { sim.step(1e300); }, "Reject numerical overflow");
    near(sim.time(), 0.0, 0.0, "Failed steps leave time unchanged");
    vectorNear(sim.state().positions[0], {1.0, 0.0, 0.0}, 0.0, "Failed steps leave state unchanged");
}

void simulationTime() {
    auto sim = simulation(examples::oscillator());
    sim.advanceTo(1.005, 0.01);
    near(sim.time(), 1.005, 0.0, "Shortened final step reaches requested time");
    check(sim.steps() == 101, "Expected full steps plus shortened final step");
    const auto steps = sim.steps();
    sim.advanceTo(sim.time(), 0.01);
    check(sim.steps() == steps, "Advancing to current time does no work");
    auto grid = simulation(examples::oscillator());
    grid.advanceTo(10.0, 0.001);
    check(grid.steps() == 10000, "No round-off-only extra endpoint step");
    rejects([&] { sim.advanceTo(0.5, 0.01); }, "Reject backwards time");
    rejects([&] { sim.advanceTo(2.0, 0.0); }, "Reject invalid maximum step");
    auto zero = simulation(examples::oscillator({1.0, 4.0, 0.0, 0.0}));
    zero.advanceTo(1.0, 0.01);
    near(zero.diagnostics().scaled_energy_change, 0.0, 0.0, "Energy diagnostic defined for zero initial energy");
}
void massGravity() {
    Model model;
    model.masses = {2.0, 5.0};
    model.gravity = {1.0, -3.0, 2.0};
    State state{{{2.0, 4.0, -1.0}, {-1.0, 2.0, 3.0}}, {{3.0, 4.0, 0.0}, {0.0, 0.0, -2.0}}};
    // T = (2/2)*25 + (5/2)*4 = 35.
    // U = -2*(2-12-2) - 5*(-1-6+6) = 29.
    near(energy(model, state, 0.0).kinetic, 35.0, 1e-14, "Unequal-mass kinetic energy");
    near(energy(model, state, 0.0).potential, 29.0, 1e-14, "Full-vector gravitational potential");
    const auto forces = evaluateForces(model, state, 0.0);
    vectorNear(forces[0], {2.0, -6.0, 4.0}, 0.0, "First gravitational force");
    vectorNear(forces[1], {5.0, -15.0, 10.0}, 0.0, "Second gravitational force");
    const auto acceleration = evaluateAccelerations(model, state, 0.0);
    vectorNear(acceleration[0], model.gravity, 1e-15, "First gravitational acceleration");
    vectorNear(acceleration[1], model.gravity, 1e-15, "Second gravitational acceleration");
}

void springRestLength() {
    Model model;
    model.masses = {2.0, 3.0};
    model.springs = {{0, 1, 6.0, 2.0}};
    State state{{Eigen::Vector3d::Zero(), {3.0, 4.0, 0.0}}, {{1.0, 2.0, 0.0}, {-2.0, 0.0, 1.0}}};
    // Length 5, extension 3: U=27, |F|=18 along the 3-4-5 triangle.
    near(energy(model, state, 0.0).potential, 27.0, 1e-13, "Spring energy uses extension");
    near(energy(model, state, 0.0).kinetic, 12.5, 1e-13, "Spring particles have unequal masses");
    const auto forces = evaluateForces(model, state, 0.0);
    vectorNear(forces[0], {10.8, 14.4, 0.0}, 1e-13, "Stretched spring first force");
    vectorNear(forces[1], {-10.8, -14.4, 0.0}, 1e-13, "Stretched spring reaction");
    const auto accelerations = evaluateAccelerations(model, state, 0.0);
    vectorNear(accelerations[0], {5.4, 7.2, 0.0}, 1e-13, "First inverse-mass acceleration");
    vectorNear(accelerations[1], {-3.6, -4.8, 0.0}, 1e-13, "Second inverse-mass acceleration");
    for (std::size_t particle = 0; particle < 2; ++particle) {
        for (int axis = 0; axis < 3; ++axis) {
            State plus = state, minus = state;
            plus.positions[particle][axis] += 1e-5;
            minus.positions[particle][axis] -= 1e-5;
            near((energy(model, plus, 0.0).potential - energy(model, minus, 0.0).potential) / 2e-5,
                 -forces[particle][axis], 1e-8, "Pair spring force equals negative potential gradient");
        }
    }
    state.positions[1] = {2.0, 0.0, 0.0};
    near(energy(model, state, 0.0).potential, 0.0, 0.0, "Rest length has zero spring energy");
    vectorNear(evaluateForces(model, state, 0.0)[0], Eigen::Vector3d::Zero(), 0.0, "Rest length has zero force");
    state.positions[1] = {1.0, 0.0, 0.0};
    near(energy(model, state, 0.0).potential, 3.0, 0.0, "Compressed spring energy");
    vectorNear(evaluateForces(model, state, 0.0)[0], {-6.0, 0.0, 0.0}, 0.0, "Compression repels endpoints");

    State initial{{Eigen::Vector3d::Zero(), {3.0, 0.0, 0.0}}, {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
    Simulation sim(model, initial);
    sim.advanceTo(1.0, 0.001);
    // Relative mode frequency uses the reduced mass: omega^2=6*(1/2+1/3)=5.
    const double separation = 2.0 + std::cos(std::sqrt(5.0));
    near(sim.state().positions[0].x(), 1.8 - 0.6 * separation, 1e-11, "Unequal-mass first analytic trajectory");
    near(sim.state().positions[1].x(), 1.8 + 0.4 * separation, 1e-11, "Unequal-mass second analytic trajectory");
    near(2.0 * sim.state().velocities[0].x() + 3.0 * sim.state().velocities[1].x(), 0.0, 1e-12,
         "Internal spring conserves total momentum");
}

void linearDamping() {
    Model model;
    model.masses = {2.0, 5.0};
    model.damping = {{0, 4.0}, {1, 4.0}};
    State state{{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}, {{3.0, 4.0, 0.0}, {6.0, 8.0, 0.0}}};
    const auto force = evaluateForces(model, state, 0.0);
    vectorNear(force[0], {-12.0, -16.0, 0.0}, 0.0, "Linear drag at speed five");
    vectorNear(force[1], {-24.0, -32.0, 0.0}, 0.0, "Doubling speed doubles drag");
    near(force[0].dot(state.velocities[0]) + force[1].dot(state.velocities[1]), -500.0, 0.0,
         "Dissipated power is minus c times speed squared");
    near(energy(model, state, 0.0).potential, 0.0, 0.0, "Damping contributes no potential energy");
    Simulation sim(model, state);
    sim.advanceTo(1.0, 0.001);
    vectorNear(sim.state().velocities[0], state.velocities[0] * std::exp(-2.0), 2e-12, "First exponential drag solution");
    vectorNear(sim.state().velocities[1], state.velocities[1] * std::exp(-0.8), 2e-12, "Second exponential drag solution");
    vectorNear(sim.state().positions[0], state.velocities[0] * (0.5 * (1.0 - std::exp(-2.0))), 2e-12,
               "Integrated drag position");
    state.velocities = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    vectorNear(evaluateForces(model, state, 0.0)[0], Eigen::Vector3d::Zero(), 0.0, "Zero speed has zero damping force");
}

void fixedPosition() {
    Model model;
    model.masses = {3.0, 2.0};
    model.springs = {{0, 1, 8.0, 2.0}};
    model.prescribed = {{0, {1.0, 2.0, 0.0}}};
    State initial{{{99.0, -4.0, 1.0}, {4.0, 2.0, 0.0}}, {{5.0, 6.0, 7.0}, Eigen::Vector3d::Zero()}};
    Simulation sim(model, initial);
    vectorNear(sim.state().positions[0], {1.0, 2.0, 0.0}, 0.0, "Fixed target overrides incompatible position");
    vectorNear(sim.state().velocities[0], Eigen::Vector3d::Zero(), 0.0, "Fixed target removes initial velocity");
    near(sim.diagnostics().energy.total(), 4.0, 0.0, "Energy baseline uses repaired fixed position");
    vectorNear(evaluateAccelerations(sim.model(), sim.state(), 0.0)[0], Eigen::Vector3d::Zero(), 0.0, "Fixed target cancels acceleration");
    sim.advanceTo(1.0, 0.001);
    vectorNear(sim.state().positions[0], {1.0, 2.0, 0.0}, 0.0, "Fixed target remains exact under spring force");
    vectorNear(sim.state().velocities[0], Eigen::Vector3d::Zero(), 0.0, "Fixed velocity remains zero");
    // Bob equilibrium is x=3, omega=sqrt(8/2)=2, initial displacement=1.
    near(sim.state().positions[1].x(), 3.0 + std::cos(2.0), 2e-11, "Rest-length anchored oscillator position");
    near(sim.state().velocities[1].x(), -2.0 * std::sin(2.0), 2e-11, "Rest-length anchored oscillator velocity");
}

void constraintEdgeCases() {
    Model model;
    model.masses = {2.0, 5.0};
    model.gravity = {0.0, -10.0, 0.0};
    model.distance_constraints = {{0, {1.0, 2.0, 3.0}, 3.0}, {1, {-2.0, 1.0, 0.0}, 2.0}};
    State state{{{5.0, 2.0, 3.0}, {-2.0, -3.0, 0.0}}, {{7.0, 4.0, 0.0}, {3.0, -8.0, 0.0}}};
    Simulation sim(model, state);
    vectorNear(sim.state().positions[0], {4.0, 2.0, 3.0}, 0.0, "Requested nonunit rod length and translated pivot");
    vectorNear(sim.state().positions[1], {-2.0, -1.0, 0.0}, 0.0, "Second requested length");
    vectorNear(sim.state().velocities[0], {0.0, 4.0, 0.0}, 0.0, "First radial velocity removed");
    vectorNear(sim.state().velocities[1], {3.0, 0.0, 0.0}, 0.0, "Second radial velocity removed");
    const auto acceleration = evaluateAccelerations(sim.model(), sim.state(), 0.0);
    vectorNear(acceleration[0], {-16.0 / 3.0, -10.0, 0.0}, 1e-14, "First centripetal acceleration");
    vectorNear(acceleration[1], {0.0, 4.5, 0.0}, 1e-14, "Second centripetal acceleration");
    sim.advanceTo(0.5, 0.001);
    near(sim.diagnostics().constraints.max_position_error, 0.0, 2e-15, "Nonunit lengths remain enforced");

    model.prescribed = {{0, Eigen::Vector3d::Zero()}};
    rejects([&] { validateModel(model); }, "Reject fixed and distance constraint on same particle");
    model.prescribed.clear();
    State singular = state;
    singular.positions[1] = model.distance_constraints[1].pivot.position(0.0);
    rejects([&] { projectConstraints(model, singular, 0.0); }, "Reject pivot with undefined radial direction");
    vectorNear(singular.positions[0], state.positions[0], 0.0, "Failed projection leaves other particles unchanged");
    rejects([&] { evaluateAccelerations(model, singular, 0.0); }, "Reject singular acceleration evaluation");

    Model spring;
    spring.masses = {1.0, 1.0};
    spring.springs = {{0, 1, 4.0, 1.0}};
    State coincident{{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}, {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
    rejects([&] { Simulation invalid(spring, coincident); }, "Reject undefined positive-rest-length spring direction");
    rejects([&] { evaluateForces(spring, coincident, 0.0); }, "Reject coincident force evaluation");
    State collision{{Eigen::Vector3d::Zero(), {2.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}}};
    Simulation approaching(spring, collision);
    rejects([&] { approaching.step(1.0); }, "Reject singular intermediate spring stage");
    near(approaching.time(), 0.0, 0.0, "Failed spring step leaves time unchanged");
    vectorNear(approaching.state().positions[1], collision.positions[1], 0.0, "Failed spring step leaves state unchanged");
    spring.springs[0].rest_length = 0.0;
    vectorNear(evaluateForces(spring, coincident, 0.0)[0], Eigen::Vector3d::Zero(), 0.0, "Coincident zero-rest-length spring has smooth zero force");
    near(energy(spring, coincident, 0.0).total(), 0.0, 0.0, "Coincident zero-rest-length spring energy");

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double invalid : {-1.0, nan, inf}) {
        spring.springs[0].rest_length = invalid;
        rejects([&] { evaluateForces(spring, coincident, 0.0); }, "Raw evaluation validates rest length");
        rejects([&] { energy(spring, coincident, 0.0); }, "Energy evaluation validates rest length");
        rejects([&] { (void)examples::oscillator({1.0, 4.0, 1.0, 0.0, invalid}); }, "Reject oscillator damping");
        rejects([&] { (void)examples::pendulum({1.0, 1.0, 9.81, 0.5, 0.0, invalid}); }, "Reject pendulum damping");
    }
    spring.springs[0] = {0, 0, 4.0, 0.0};
    rejects([&] { validateModel(spring); }, "Reject self spring");
    spring.springs[0] = {0, 2, 4.0, 0.0};
    rejects([&] { validateModel(spring); }, "Reject spring endpoint outside model");
    for (double invalid : {0.0, -1.0, nan, inf}) {
        spring.springs[0] = {0, 1, invalid, 0.0};
        rejects([&] { validateModel(spring); }, "Reject invalid pair spring stiffness");
    }
    spring.springs.clear();
    spring.damping = {{2, 0.5}};
    rejects([&] { validateModel(spring); }, "Reject damping particle index");
    spring.damping.clear();
    spring.prescribed = {{2, Eigen::Vector3d::Zero()}};
    rejects([&] { validateModel(spring); }, "Reject fixed-position index");
    spring.prescribed = {{0, {nan, 0.0, 0.0}}};
    rejects([&] { validateModel(spring); }, "Reject nonfinite fixed target");
    spring.prescribed = {{0, Eigen::Vector3d::Zero()}, {0, {1.0, 0.0, 0.0}}};
    rejects([&] { validateModel(spring); }, "Reject competing fixed targets");
    spring.prescribed.clear();
    for (double invalid : {0.0, -1.0, nan, inf}) {
        spring.distance_constraints = {{0, Eigen::Vector3d::Zero(), invalid}};
        rejects([&] { validateModel(spring); }, "Reject invalid distance length");
    }
    spring.distance_constraints.clear();
    spring.masses[0] = 0.0;
    rejects([&] { evaluateForces(spring, coincident, 0.0); }, "Raw force evaluation rejects zero mass");
    spring.masses[0] = 1.0;
    coincident.velocities[0].x() = inf;
    rejects([&] { energy(spring, coincident, 0.0); }, "Energy rejects nonfinite velocity");
}

void dampedExamples() {
    for (double damping : {1.0, 8.0, 10.0}) {
        const examples::OscillatorConfig config{2.0, 8.0, 1.0, 0.2, damping};
        auto sim = simulation(examples::oscillator(config));
        sim.advanceTo(2.0, 0.001);
        double x, v;
        if (damping == 1.0) {
            const double beta = 0.25, omega = std::sqrt(4.0 - beta * beta);
            const double b = (0.2 + beta) / omega;
            x = std::exp(-beta * 2.0) * (std::cos(omega * 2.0) + b * std::sin(omega * 2.0));
            v = -beta * x + std::exp(-beta * 2.0) * omega * (-std::sin(omega * 2.0) + b * std::cos(omega * 2.0));
        } else if (damping == 8.0) {
            x = (1.0 + 2.2 * 2.0) * std::exp(-4.0);
            v = (2.2 - 2.0 * (1.0 + 2.2 * 2.0)) * std::exp(-4.0);
        } else {
            // Characteristic roots -1 and -4; A+B=1 and -A-4B=0.2.
            const double a = 1.4, b = -0.4;
            x = a * std::exp(-2.0) + b * std::exp(-8.0);
            v = -a * std::exp(-2.0) - 4.0 * b * std::exp(-8.0);
        }
        near(sim.state().positions[0].x(), x, 2e-11, "Damped oscillator analytic position");
        near(sim.state().velocities[0].x(), v, 2e-11, "Damped oscillator analytic velocity");
        check(sim.diagnostics().energy_change < 0.0, "Damped oscillator loses mechanical energy");
    }
    const examples::PendulumConfig config{2.0, 1.3, 9.81, 1.0, 0.2, 2.0};
    const auto reference = angularReference(3.0, config);
    auto sim = simulation(examples::pendulum(config));
    double previous_energy = sim.diagnostics().energy.total();
    for (int i = 0; i < 1500; ++i) {
        sim.step(0.002);
        const double current_energy = sim.diagnostics().energy.total();
        check(current_energy <= previous_energy + 1e-10, "Damped pendulum mechanical energy decreases");
        previous_energy = current_energy;
    }
    const auto p = sim.state().positions[0], v = sim.state().velocities[0];
    near(std::atan2(p.x(), -p.y()), reference.first, 2e-10, "Damped pendulum angular reference position");
    near((p.x() * v.y() - p.y() * v.x()) / (config.length * config.length), reference.second, 2e-10,
         "Damped pendulum angular reference velocity");
}
} // namespace

int main(int argc, char** argv) {
    return test::run(argc, argv, {
        {"forces_energy", forcesEnergy}, {"initialization", initialization},
        {"oscillator_accuracy", oscillatorAccuracy}, {"oscillator_convergence", oscillatorConvergence},
        {"pendulum_small_angle", pendulumSmallAngle}, {"pendulum_nonlinear", pendulumNonlinear},
        {"pendulum_period", pendulumPeriod}, {"conservation", conservation},
        {"validation", validation}, {"simulation_time", simulationTime},
        {"mass_gravity", massGravity}, {"spring_rest_length", springRestLength},
        {"linear_damping", linearDamping}, {"fixed_position", fixedPosition},
        {"constraint_edge_cases", constraintEdgeCases}, {"damped_examples", dampedExamples}});
}
