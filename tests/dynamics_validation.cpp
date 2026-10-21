// Numerical validation of the V1 dynamics features. It measures convergence
// order of fixed-step RK4 with coupled, moving, and rheonomic constraints and
// with time-dependent potentials, and the accuracy and cost of adaptive
// Dormand-Prince integration against fixed RK4. It prints every measurement,
// optionally writes them to CSV, and fails if an acceptance criterion is missed.
#include "examples.hpp"
#include "physics/simulation.hpp"
#include "references.hpp"
#include "test_support.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <locale>
#include <string>
#include <vector>

namespace {
using namespace physics;
using test::check;
using test::pi;

struct Row {
    std::string system;
    std::string method; // rk4 (parameter = dt) or adaptive (parameter = tolerance)
    double parameter;
    double error;
    double energy_metric; // |energy balance error| or |energy change|, J
    double constraint_error;
    std::uint64_t evaluations;
    double order = 0.0;   // log2 of the error ratio to the previous (coarser) row
};

// Generalized coordinates measured from a simulation, compared with a reference.
struct Convergence {
    std::string system;
    examples::Example example;
    test::Ode ode;
    Eigen::VectorXd initial;
    double end_time;
    std::vector<double> steps;
    double reference_step;
    // Phase-space error of the simulation against the reference at end_time.
    std::function<double(const Simulation&, const Eigen::VectorXd&)> error;
};

std::vector<Row> convergence(const Convergence& c) {
    const auto reference = test::integrate(c.ode, c.initial, 0.0, c.end_time, c.reference_step);
    const auto coarse = test::integrate(c.ode, c.initial, 0.0, c.end_time, 2.0 * c.reference_step);
    const double reference_error = (reference - coarse).lpNorm<Eigen::Infinity>();
    check(reference_error < 1e-11, c.system + ": reference has not converged");
    std::cout << c.system << " reference refinement difference=" << reference_error << '\n';
    std::vector<Row> rows;
    for (double dt : c.steps) {
        Simulation sim(c.example.model, c.example.initial_state);
        sim.advanceTo(c.end_time, dt);
        const auto d = sim.diagnostics();
        Row row{c.system, "rk4", dt, c.error(sim, reference), std::abs(d.energy_balance_error),
                d.constraints.max_position_error, d.steps.evaluations};
        if (!rows.empty()) row.order = std::log2(rows.back().error / row.error);
        rows.push_back(row);
    }
    return rows;
}

double angleError(double angle, double reference) {
    return std::abs(std::remainder(angle - reference, 2.0 * pi));
}

std::vector<Row> fixedStepSweep() {
    std::vector<Row> rows;
    const auto append = [&](const std::vector<Row>& more) { rows.insert(rows.end(), more.begin(), more.end()); };

    const examples::DoublePendulumConfig dp{1.0, 1.5, 1.0, 0.8, 9.81, 1.2, 0.6, 0.0, 0.5};
    append(convergence({"double-pendulum", examples::doublePendulum(dp), test::doublePendulumOde(dp),
        test::vector({dp.angle1, dp.angle2, dp.angular_velocity1, dp.angular_velocity2}), 3.0,
        {0.008, 0.004, 0.002, 0.001}, 1e-5, [](const Simulation& sim, const Eigen::VectorXd& y) {
            const auto& q = sim.state().positions;
            const auto& v = sim.state().velocities;
            const Eigen::Vector3d d = q[1] - q[0], w = v[1] - v[0];
            const double w1 = (q[0].x() * v[0].y() - q[0].y() * v[0].x()) / q[0].squaredNorm();
            const double w2 = (d.x() * w.y() - d.y() * w.x()) / d.squaredNorm();
            const double scale = std::sqrt(1.0 / 9.81); // rad/s -> rad over the pendulum time scale
            return std::sqrt(std::pow(angleError(std::atan2(q[0].x(), -q[0].y()), y[0]), 2)
                             + std::pow(angleError(std::atan2(d.x(), -d.y()), y[1]), 2)
                             + std::pow((w1 - y[2]) * scale, 2) + std::pow((w2 - y[3]) * scale, 2));
        }}));

    const examples::KapitzaPendulumConfig kp{};
    append(convergence({"kapitza-pendulum", examples::kapitzaPendulum(kp), test::kapitzaOde(kp),
        test::vector({kp.angle, kp.angular_velocity}), 1.0, {4e-4, 2e-4, 1e-4, 5e-5}, 2.5e-6,
        [kp](const Simulation& sim, const Eigen::VectorXd& y) {
            const double t = sim.time();
            const Eigen::Vector3d pivot{0.0, kp.pivot_amplitude * std::cos(kp.pivot_frequency * t), 0.0};
            const Eigen::Vector3d pivot_velocity{0.0, -kp.pivot_amplitude * kp.pivot_frequency
                                                          * std::sin(kp.pivot_frequency * t), 0.0};
            const Eigen::Vector3d d = sim.state().positions[0] - pivot;
            const Eigen::Vector3d w = sim.state().velocities[0] - pivot_velocity;
            const double rate = (d.x() * w.y() - d.y() * w.x()) / d.squaredNorm();
            return std::hypot(angleError(std::atan2(d.x(), -d.y()), y[0]), (rate - y[1]) / kp.pivot_frequency);
        }}));

    const examples::RotatingHoopConfig hp{};
    append(convergence({"rotating-hoop", examples::rotatingHoop(hp), test::rotatingHoopOde(hp),
        test::vector({hp.angle, hp.angular_velocity}), 3.0, {0.01, 0.005, 0.0025, 0.00125}, 2.5e-5,
        [hp](const Simulation& sim, const Eigen::VectorXd& y) {
            const double phase = hp.rotation_rate * sim.time();
            const Eigen::Vector3d u{std::cos(phase), 0.0, -std::sin(phase)};
            const Eigen::Vector3d& q = sim.state().positions[0];
            const double theta = std::atan2(q.dot(u), -q.y());
            const Eigen::Vector3d relative = sim.state().velocities[0]
                - Eigen::Vector3d{0.0, hp.rotation_rate, 0.0}.cross(q);
            const double rate = relative.dot(std::cos(theta) * u + std::sin(theta) * Eigen::Vector3d::UnitY()) / hp.radius;
            return std::hypot(angleError(theta, y[0]), (rate - y[1]) / hp.rotation_rate);
        }}));

    const examples::ParametricOscillatorConfig po{};
    append(convergence({"parametric-oscillator", examples::parametricOscillator(po), test::parametricOde(po),
        test::vector({po.displacement, po.velocity}), 40.0, {0.04, 0.02, 0.01, 0.005}, 1e-4,
        [po](const Simulation& sim, const Eigen::VectorXd& y) {
            // Relative to the amplitude scale, which grows by parametric resonance.
            return std::hypot(sim.state().positions[0].x() - y[0],
                              (sim.state().velocities[0].x() - y[1]) / po.natural_frequency) / std::hypot(y[0], y[1]);
        }}));
    return rows;
}

// One Kepler period from periapsis: error is the distance from the start point.
Row keplerRun(double eccentricity, const std::string& method, double parameter) {
    const examples::KeplerConfig config{1.0, 1.0, 1.0, eccentricity};
    const auto example = examples::kepler(config);
    Simulation sim(example.model, example.initial_state);
    const double period = 2.0 * pi;
    if (method == "rk4") sim.advanceTo(period, parameter);
    else sim.advanceAdaptive(period, {parameter, parameter});
    const auto d = sim.diagnostics();
    return {"kepler-e" + std::to_string(eccentricity).substr(0, 3), method, parameter,
            (sim.state().positions[0] - example.initial_state.positions[0]).norm(), std::abs(d.energy_change),
            d.constraints.max_position_error, d.steps.evaluations};
}

std::vector<Row> adaptiveSweep() {
    std::vector<Row> rows;
    for (double e : {0.6, 0.9}) {
        for (double dt : {0.004, 0.002, 0.001, 0.0005}) {
            rows.push_back(keplerRun(e, "rk4", dt));
            if (rows.size() > 1 && rows[rows.size() - 2].method == "rk4" && rows[rows.size() - 2].system == rows.back().system) {
                rows.back().order = std::log2(rows[rows.size() - 2].error / rows.back().error);
            }
        }
        for (double tolerance : {1e-6, 1e-8, 1e-10, 1e-12}) {
            rows.push_back(keplerRun(e, "adaptive", tolerance));
            if (rows[rows.size() - 2].method == "adaptive") {
                rows.back().order = std::log10(rows[rows.size() - 2].error / rows.back().error) / 2.0;
            }
        }
    }
    // Figure-eight closure after one period.
    const auto example = examples::threeBody();
    for (double tolerance : {1e-8, 1e-10, 1e-12}) {
        Simulation sim(example.model, example.initial_state);
        sim.advanceAdaptive(examples::figure_eight_period, {tolerance, tolerance});
        double closure = 0.0;
        for (std::size_t i = 0; i < 3; ++i) {
            closure = std::max(closure, (sim.state().positions[i] - example.initial_state.positions[i]).norm());
        }
        const auto d = sim.diagnostics();
        rows.push_back({"three-body", "adaptive", tolerance, closure, std::abs(d.energy_change), 0.0,
                        d.steps.evaluations});
    }
    return rows;
}

Row find(const std::vector<Row>& rows, const std::string& system, const std::string& method, double parameter) {
    for (const auto& row : rows) {
        if (row.system == system && row.method == method && row.parameter == parameter) return row;
    }
    throw std::runtime_error("Missing row " + system + " " + method);
}

void validate(const std::vector<Row>& fixed, const std::vector<Row>& adaptive) {
    for (const auto& row : fixed) {
        check(std::isfinite(row.error) && row.error > 0.0, row.system + ": non-finite or zero error");
        if (row.order != 0.0) check(row.order >= 3.6, row.system + ": RK4 convergence order below 3.6");
        check(row.constraint_error <= 1e-13, row.system + ": constraint error above 1e-13");
    }
    for (const auto& system : {"double-pendulum", "kapitza-pendulum", "rotating-hoop"}) {
        double finest = 1.0;
        for (const auto& row : fixed) if (row.system == system) finest = row.error;
        check(finest <= 1e-9, std::string(system) + ": finest-step error above 1e-9");
    }
    for (const auto& system : {"kepler-e0.6", "kepler-e0.9"}) {
        double previous = 1.0;
        for (double tolerance : {1e-6, 1e-8, 1e-10, 1e-12}) {
            const Row row = find(adaptive, system, "adaptive", tolerance);
            check(row.error < previous, std::string(system) + ": adaptive error must fall with tolerance");
            check(row.error <= 1e4 * tolerance, std::string(system) + ": adaptive error above 1e4 x tolerance");
            previous = row.error;
        }
        const Row rk4 = find(adaptive, system, "rk4", 0.001);
        const Row dopri = find(adaptive, system, "adaptive", 1e-12);
        check(dopri.error < rk4.error && 5 * dopri.evaluations < rk4.evaluations,
              std::string(system) + ": adaptive at 1e-12 must beat RK4 at dt=0.001 in accuracy at a fifth of the cost");
    }
    check(find(adaptive, "three-body", "adaptive", 1e-12).error <= 1e-6, "Figure-eight must close to 1e-6");
}

void print(const std::vector<Row>& rows) {
    std::cout << std::setprecision(4);
    for (const auto& row : rows) {
        std::cout << row.system << ' ' << row.method << ' ' << row.parameter << " error=" << row.error
                  << " energy=" << row.energy_metric << " constraint=" << row.constraint_error
                  << " evaluations=" << row.evaluations << " order=" << row.order << '\n';
    }
}

void writeCsv(const std::filesystem::path& path, const std::vector<Row>& rows) {
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path);
    check(static_cast<bool>(output), "Cannot open dynamics validation CSV");
    output.imbue(std::locale::classic());
    output << std::setprecision(17) << "system,method,parameter,error,energy_metric,constraint_error,evaluations,order\n";
    for (const auto& row : rows) {
        output << row.system << ',' << row.method << ',' << row.parameter << ',' << row.error << ','
               << row.energy_metric << ',' << row.constraint_error << ',' << row.evaluations << ',' << row.order << '\n';
    }
    output.close();
    check(static_cast<bool>(output), "Cannot finish dynamics validation CSV");
}
} // namespace

int main(int argc, char** argv) {
    try {
        check(argc <= 2, "Usage: physics_dynamics_numerics [output.csv]");
        const auto fixed = fixedStepSweep();
        const auto adaptive = adaptiveSweep();
        print(fixed);
        print(adaptive);
        if (argc == 2) {
            auto all = fixed;
            all.insert(all.end(), adaptive.begin(), adaptive.end());
            writeCsv(argv[1], all);
        }
        validate(fixed, adaptive);
        std::cout << "Dynamics accuracy criteria passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
