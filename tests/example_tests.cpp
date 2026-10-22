// Validation of the example systems against independent references: exact
// solutions, generalized-coordinate equations integrated separately, and
// conservation laws. Each test prints its measured errors.
#include "examples.hpp"
#include "physics/simulation.hpp"
#include "references.hpp"
#include "test_support.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

namespace {
using namespace physics;
using test::check;
using test::near;
using test::pi;
using test::vectorNear;

Simulation simulation(const examples::Example& example) {
    return Simulation(example.model, example.initial_state);
}

void drivenOscillator() {
    const examples::DrivenOscillatorConfig c{1.0, 4.0, 0.2, 1.0, 1.8, 0.3, -0.1};
    // Exact solution: steady response plus underdamped transient.
    const double detuning = c.stiffness - c.mass * c.drive_frequency * c.drive_frequency;
    const double amplitude = c.force_amplitude / std::hypot(detuning, c.damping * c.drive_frequency);
    const double lag = std::atan2(c.damping * c.drive_frequency, detuning);
    const double beta = c.damping / (2.0 * c.mass);
    const double omega = std::sqrt(c.stiffness / c.mass - beta * beta);
    const double c1 = c.displacement - amplitude * std::cos(lag);
    const double c2 = (c.velocity - amplitude * c.drive_frequency * std::sin(lag) + beta * c1) / omega;
    const auto exact = [&](double t) {
        const double decay = std::exp(-beta * t);
        const double x = amplitude * std::cos(c.drive_frequency * t - lag)
            + decay * (c1 * std::cos(omega * t) + c2 * std::sin(omega * t));
        const double v = -amplitude * c.drive_frequency * std::sin(c.drive_frequency * t - lag)
            + decay * ((-beta * c1 + omega * c2) * std::cos(omega * t) - (beta * c2 + omega * c1) * std::sin(omega * t));
        return std::make_pair(x, v);
    };
    auto fixed = simulation(examples::drivenOscillator(c));
    fixed.advanceTo(20.0, 0.001);
    auto adaptive = simulation(examples::drivenOscillator(c));
    adaptive.advanceAdaptive(20.0, {1e-11, 1e-11});
    const auto [x, v] = exact(20.0);
    for (auto* sim : {&fixed, &adaptive}) {
        const double error = std::hypot(sim->state().positions[0].x() - x, sim->state().velocities[0].x() - v);
        near(error, 0.0, sim == &fixed ? 1e-11 : 1e-9, "Driven oscillator exact solution");
        near(sim->diagnostics().energy_balance_error, 0.0, sim == &fixed ? 1e-11 : 1e-8, "Driven energy balance");
        std::cout << (sim == &fixed ? "rk4" : "adaptive") << " error=" << error
                  << " work=" << sim->diagnostics().external_work
                  << " balance=" << sim->diagnostics().energy_balance_error << '\n';
    }
    check(adaptive.steps() < fixed.steps() / 5, "Adaptive integration takes far fewer steps");
}

void parametricOscillator() {
    const examples::ParametricOscillatorConfig resonant{};
    auto sim = simulation(examples::parametricOscillator(resonant));
    sim.advanceTo(40.0, 0.002);
    const auto y = test::integrate(test::parametricOde(resonant),
                                   test::vector({resonant.displacement, resonant.velocity}), 0.0, 40.0, 1e-4);
    near(sim.state().positions[0].x(), y[0], 1e-11, "Parametric oscillator matches reference position");
    near(sim.state().velocities[0].x(), y[1], 1e-11, "Parametric oscillator matches reference velocity");
    near(sim.diagnostics().energy_balance_error, 0.0, 1e-14, "Time-dependent potential energy balance");
    // Parametric resonance (Landau Sec. 27): near gamma = 2 w0 the amplitude
    // grows; first-order theory gives the rate s = h w0 / 4.
    const auto envelope = [](Simulation& s, double until) {
        double peak = 0.0;
        while (s.time() < until - 1e-9) {
            s.advanceTo(std::min(until, s.time() + 0.01), 0.002);
            if (s.time() > until - 2.0 * pi) peak = std::max(peak, std::abs(s.state().positions[0].x()));
        }
        return peak;
    };
    // Measure late, once the decaying Floquet mode is negligible.
    auto growing = simulation(examples::parametricOscillator(resonant));
    const double early = envelope(growing, 60.0), late = envelope(growing, 120.0);
    const double rate = std::log(late / early) / 60.0;
    near(rate, resonant.modulation * resonant.natural_frequency / 4.0, 0.02 * 0.025,
         "Parametric resonance growth rate h w0 / 4");
    auto detuned = simulation(examples::parametricOscillator({1.0, 1.0, 0.1, 1.3}));
    check(envelope(detuned, 80.0) < 1.3 * resonant.displacement, "Away from resonance the amplitude stays bounded");
    std::cout << "growth rate=" << rate << " (theory 0.025)\n";
}

double kapitzaAngle(const Simulation& sim, const examples::KapitzaPendulumConfig& c) {
    const Eigen::Vector3d pivot{0.0, c.pivot_amplitude * std::cos(c.pivot_frequency * sim.time()), 0.0};
    const Eigen::Vector3d d = sim.state().positions.back() - pivot;
    return std::atan2(d.x(), -d.y());
}

void kapitzaPendulum() {
    const examples::KapitzaPendulumConfig c{};
    const auto reference = test::integrate(test::kapitzaOde(c), test::vector({c.angle, c.angular_velocity}),
                                           0.0, 1.0, 1e-5);
    auto sim = simulation(examples::kapitzaPendulum(c));
    sim.advanceTo(1.0, 1e-4);
    const double error = std::abs(kapitzaAngle(sim, c) - reference[0]);
    near(error, 0.0, 1e-8, "Moving-pivot pendulum matches the angular equation");
    near(sim.diagnostics().scaled_energy_balance_error, 0.0, 5e-9, "Moving-support energy balance");
    check(sim.diagnostics().external_work > 1.0, "The moving pivot does work on the bob");

    // The same support modelled as a prescribed particle with a link.
    auto equivalent = examples::kapitzaPendulum(c);
    const Motion pivot = equivalent.model.distance_constraints[0].pivot;
    equivalent.model.masses = {1.0, c.mass};
    equivalent.model.distance_constraints.clear();
    equivalent.model.prescribed = {{0, pivot}};
    equivalent.model.links = {{0, 1, c.length}};
    equivalent.initial_state.positions.insert(equivalent.initial_state.positions.begin(), pivot.position(0.0));
    equivalent.initial_state.velocities.insert(equivalent.initial_state.velocities.begin(), pivot.velocity(0.0));
    auto linked = simulation(equivalent);
    linked.advanceTo(1.0, 1e-4);
    vectorNear(linked.state().positions[1], sim.state().positions[0], 1e-11,
               "Prescribed particle plus link equals a moving pivot");

    // Kapitza stabilization (Landau Sec. 30): the inverted pendulum is stable
    // when (a w)^2 > 2 g l and falls otherwise.
    const auto maxDeviation = [&](const examples::KapitzaPendulumConfig& config) {
        auto run = simulation(examples::kapitzaPendulum(config));
        double deviation = 0.0;
        for (int k = 1; k <= 400; ++k) {
            run.advanceAdaptive(0.025 * k, {1e-9, 1e-9});
            const double theta = kapitzaAngle(run, config);
            deviation = std::max(deviation, std::abs(std::remainder(theta - pi, 2.0 * pi)));
        }
        return deviation;
    };
    const double stable = maxDeviation(c);
    examples::KapitzaPendulumConfig slow = c;
    slow.pivot_frequency = 60.0; // a w = 3 < sqrt(2 g l) = 4.43
    const double unstable = maxDeviation(slow);
    check(stable < 0.3, "Fast pivot oscillation stabilizes the inverted pendulum");
    check(unstable > 1.5, "Slow pivot oscillation lets the inverted pendulum fall");
    std::cout << "reference error=" << error << " inverted deviation: stable=" << stable
              << " unstable=" << unstable << '\n';
}

void doublePendulum() {
    const examples::DoublePendulumConfig c{1.0, 1.5, 1.0, 0.8, 9.81, 1.2, 0.6, 0.0, 0.5};
    const auto reference = test::integrate(test::doublePendulumOde(c),
        test::vector({c.angle1, c.angle2, c.angular_velocity1, c.angular_velocity2}), 0.0, 3.0, 2e-5);
    const auto angles = [](const Simulation& sim) {
        const auto& q = sim.state().positions;
        const Eigen::Vector3d d = q[1] - q[0];
        return std::make_pair(std::atan2(q[0].x(), -q[0].y()), std::atan2(d.x(), -d.y()));
    };
    auto fixed = simulation(examples::doublePendulum(c));
    fixed.advanceTo(3.0, 0.001);
    auto adaptive = simulation(examples::doublePendulum(c));
    adaptive.advanceAdaptive(3.0, {1e-11, 1e-11});
    for (auto* sim : {&fixed, &adaptive}) {
        const auto [a1, a2] = angles(*sim);
        const double error = std::hypot(a1 - reference[0], a2 - reference[1]);
        near(error, 0.0, 1e-8, "Double pendulum matches Lagrange's equations");
        const auto d = sim->diagnostics();
        near(d.energy_change, 0.0, 1e-9, "Double pendulum conserves energy");
        near(d.constraints.max_position_error, 0.0, 1e-14, "Both rods stay rigid");
        near(d.constraints.max_velocity_error, 0.0, 1e-13, "Both rod velocity constraints hold");
        std::cout << (sim == &fixed ? "rk4" : "adaptive") << " angle error=" << error
                  << " energy change=" << d.energy_change << " steps=" << sim->steps() << '\n';
    }
}

void foldedChain() {
    // An ideal chain: fine links and a narrow fold approach the continuum model.
    examples::FoldedChainConfig c;
    c.links = 60.0;
    c.gap = 0.06;
    const auto example = examples::foldedChain(c);
    const std::size_t tip = example.initial_state.positions.size() - 1;
    const double start = example.initial_state.positions[tip].y();
    const double release = evaluateAccelerations(example.model, example.initial_state, 0.0)[tip].y();

    // Times at which the free end has fallen each depth, interpolated between samples.
    auto sim = simulation(example);
    const std::vector<double> depths{0.5, 1.0, 1.5};
    std::vector<double> times;
    double previous_time = 0.0, previous_drop = 0.0;
    while (times.size() < depths.size()) {
        sim.advanceAdaptive(sim.time() + 5e-4, {1e-10, 1e-10});
        const double drop = start - sim.state().positions[tip].y();
        while (times.size() < depths.size() && drop >= depths[times.size()]) {
            const double fraction = (depths[times.size()] - previous_drop) / (drop - previous_drop);
            times.push_back(previous_time + fraction * (sim.time() - previous_time));
        }
        previous_time = sim.time();
        previous_drop = drop;
    }
    for (std::size_t i = 0; i < depths.size(); ++i) {
        const double reference = test::foldedChainFallTime(c.length, c.gravity, depths[i]);
        const double free_fall = std::sqrt(2.0 * depths[i] / c.gravity);
        near(times[i] / reference, 1.0, 0.015, "Free end follows the energy-conserving chain model");
        std::cout << "depth " << depths[i] << " m: t=" << times[i] << " s, model " << reference
                  << " s, free fall " << free_fall << " s\n";
    }
    near(release, -c.gravity, 0.05, "Free end starts in free fall");
    check(times.back() < 0.95 * std::sqrt(2.0 * depths.back() / c.gravity), "Free end falls faster than g");
    const auto d = sim.diagnostics();
    near(d.energy_change, 0.0, 1e-8, "Ideal chain conserves energy");
    near(d.constraints.max_position_error, 0.0, 1e-13, "Links stay rigid");
    std::cout << "release acceleration=" << release << " energy change=" << d.energy_change
              << " steps=" << sim.steps() << '\n';
}

void rotatingHoop() {
    const examples::RotatingHoopConfig c{};
    const double g = c.gravity, r = c.radius, spin = c.rotation_rate;
    // Bead angle and rate from the engine state, in the hoop's rotating frame.
    const auto polar = [&](const Simulation& sim) {
        const double phase = spin * sim.time();
        const Eigen::Vector3d u{std::cos(phase), 0.0, -std::sin(phase)};
        const Eigen::Vector3d& q = sim.state().positions[0];
        const double theta = std::atan2(q.dot(u), -q.y());
        const Eigen::Vector3d relative = sim.state().velocities[0] - Eigen::Vector3d{0.0, spin, 0.0}.cross(q);
        const Eigen::Vector3d tangent = std::cos(theta) * u + std::sin(theta) * Eigen::Vector3d::UnitY();
        return std::make_pair(theta, relative.dot(tangent) / r);
    };
    // Jacobi integral per unit mass, conserved although the energy is not.
    const auto jacobi = [&](double theta, double rate) {
        return 0.5 * r * r * rate * rate - 0.5 * r * r * spin * spin * std::sin(theta) * std::sin(theta)
            - g * r * std::cos(theta);
    };
    const auto reference = test::integrate(test::rotatingHoopOde(c), test::vector({c.angle, c.angular_velocity}),
                                           0.0, 3.0, 1e-4);
    auto sim = simulation(examples::rotatingHoop(c));
    const double initial_jacobi = jacobi(c.angle, c.angular_velocity);
    double jacobi_drift = 0.0;
    for (int k = 1; k <= 300; ++k) {
        sim.advanceTo(0.01 * k, 0.001);
        const auto [theta, rate] = polar(sim);
        jacobi_drift = std::max(jacobi_drift, std::abs(jacobi(theta, rate) - initial_jacobi));
    }
    const auto [theta, rate] = polar(sim);
    near(theta, reference[0], 1e-9, "Bead angle matches the reduced equation");
    near(rate, reference[1], 1e-8, "Bead rate matches the reduced equation");
    near(jacobi_drift, 0.0, 1e-9, "Jacobi integral is conserved");
    const auto d = sim.diagnostics();
    check(std::abs(d.energy_change) > 1.0, "The spinning hoop changes the bead's energy");
    near(d.energy_balance_error, 0.0, 1e-9, "Hoop work balances the energy change");
    near(d.constraints.max_position_error, 0.0, 1e-14, "Bead stays on the rotating hoop");

    // Off-axis relative equilibrium cos(theta0) = g / (R Omega^2).
    examples::RotatingHoopConfig resting = c;
    resting.angle = std::acos(g / (r * spin * spin));
    auto equilibrium = simulation(examples::rotatingHoop(resting));
    equilibrium.advanceAdaptive(5.0, {1e-11, 1e-11});
    near(polar(equilibrium).first, resting.angle, 1e-8, "Bead rests at the off-axis equilibrium");
    std::cout << "angle error=" << std::abs(theta - reference[0]) << " jacobi drift=" << jacobi_drift
              << " hoop work=" << d.external_work << '\n';
}

void kepler() {
    const examples::KeplerConfig c{};
    const double mu = c.gravitational_parameter, a = c.semi_major_axis, e = c.eccentricity;
    const double period = 2.0 * pi * std::sqrt(a * a * a / mu);
    const auto example = examples::kepler(c);
    // Laplace-Runge-Lenz vector per unit mass: v x h - mu q / |q|.
    const auto lenz = [&](const State& s) {
        const Eigen::Vector3d h = s.positions[0].cross(s.velocities[0]);
        return Eigen::Vector3d(s.velocities[0].cross(h) - mu * s.positions[0] / s.positions[0].norm());
    };
    auto adaptive = simulation(example);
    adaptive.advanceAdaptive(period, {1e-12, 1e-12});
    const auto d = adaptive.diagnostics();
    const double return_error = (adaptive.state().positions[0] - example.initial_state.positions[0]).norm();
    near(return_error, 0.0, 1e-8, "Orbit closes after the Kepler period");
    near(d.energy.total(), -mu * c.mass / (2.0 * a), 1e-10, "Orbital energy -mu m / 2a");
    vectorNear(d.momentum.angular, momentum(example.model, example.initial_state).angular, 1e-10,
               "Angular momentum is conserved");
    vectorNear(lenz(adaptive.state()), Eigen::Vector3d{mu * e, 0.0, 0.0}, 1e-9,
               "Laplace-Runge-Lenz vector is conserved");
    const auto& s = adaptive.statistics();
    check(s.max_step > 8.0 * s.min_step, "Adaptive steps shrink near periapsis");

    auto fixed = simulation(example);
    fixed.advanceTo(period, 0.001);
    const double fixed_error = (fixed.state().positions[0] - example.initial_state.positions[0]).norm();
    check(return_error < fixed_error && s.evaluations < fixed.statistics().evaluations,
          "Adaptive is more accurate than fixed RK4 for fewer evaluations");
    std::cout << "adaptive return error=" << return_error << " evaluations=" << s.evaluations
              << " step range=[" << s.min_step << ", " << s.max_step << "]; rk4 dt=0.001 return error="
              << fixed_error << " evaluations=" << fixed.statistics().evaluations << '\n';
}

void threeBody() {
    const auto example = examples::threeBody();
    const auto initial = momentum(example.model, example.initial_state);
    auto sim = simulation(example);
    sim.advanceAdaptive(examples::figure_eight_period / 3.0, {1e-12, 1e-12});
    // Choreography: after a third of the period the bodies have exchanged places.
    const auto& q = sim.state().positions;
    const auto& q0 = example.initial_state.positions;
    const double shift = std::max({(q[0] - q0[2]).norm(), (q[1] - q0[0]).norm(), (q[2] - q0[1]).norm()});
    near(shift, 0.0, 1e-6, "Each body follows the next one's path a third of a period later");
    sim.advanceAdaptive(examples::figure_eight_period, {1e-12, 1e-12});
    double closure = 0.0;
    for (std::size_t i = 0; i < 3; ++i) closure = std::max(closure, (sim.state().positions[i] - q0[i]).norm());
    near(closure, 0.0, 1e-6, "Figure-eight orbit is periodic");
    const auto d = sim.diagnostics();
    near(d.energy_change, 0.0, 1e-10, "Three-body energy is conserved");
    vectorNear(d.momentum.linear, initial.linear, 1e-13, "Three-body linear momentum is conserved");
    vectorNear(d.momentum.angular, initial.angular, 1e-11, "Three-body angular momentum is conserved");

    // Scaling: G m = 4 halves the period at the same positions.
    auto scaled = simulation(examples::threeBody({2.0, 2.0}));
    scaled.advanceAdaptive(examples::figure_eight_period / 2.0, {1e-12, 1e-12});
    vectorNear(scaled.state().positions[2], q0[2], 1e-6, "Figure-eight period scales as 1/sqrt(G m)");
    std::cout << "third-period shift=" << shift << " closure=" << closure
              << " energy change=" << d.energy_change << '\n';
}

void cyclotron() {
    const examples::CyclotronConfig c{};
    const double omega = c.charge * c.field / c.mass, radius = c.mass * c.speed / (c.charge * c.field);
    const auto exact = [&](double t) {
        return Eigen::Vector3d{radius * std::sin(omega * t), -radius * (1.0 - std::cos(omega * t)), c.axial_velocity * t};
    };
    auto fixed = simulation(examples::cyclotron(c));
    fixed.advanceTo(10.0, 0.001);
    auto adaptive = simulation(examples::cyclotron(c));
    adaptive.advanceAdaptive(10.0, {1e-11, 1e-11});
    for (auto* sim : {&fixed, &adaptive}) {
        const double error = (sim->state().positions[0] - exact(10.0)).norm();
        near(error, 0.0, sim == &fixed ? 1e-10 : 1e-8, "Helical trajectory");
        near(sim->diagnostics().external_work, 0.0, 1e-13, "Magnetic force does no work");
        near(sim->diagnostics().energy_change, 0.0, sim == &fixed ? 1e-12 : 1e-9, "Kinetic energy is constant");
        std::cout << (sim == &fixed ? "rk4" : "adaptive") << " helix error=" << error << '\n';
    }
}
} // namespace

int main(int argc, char** argv) {
    return test::run(argc, argv, {
        {"driven_oscillator", drivenOscillator}, {"parametric_oscillator", parametricOscillator},
        {"kapitza_pendulum", kapitzaPendulum}, {"double_pendulum", doublePendulum},
        {"folded_chain", foldedChain}, {"rotating_hoop", rotatingHoop}, {"kepler", kepler}, {"three_body", threeBody},
        {"cyclotron", cyclotron}});
}
