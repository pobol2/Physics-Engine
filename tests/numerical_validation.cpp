#include "examples.hpp"
#include "physics/simulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace physics;
constexpr double pi = 3.14159265358979323846;
constexpr std::array<double, 4> timesteps{0.04, 0.02, 0.01, 0.005};
constexpr double duration = 50.0;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

Simulation makeSimulation(examples::Example example) {
    return Simulation(std::move(example.model), std::move(example.initial_state));
}

// Independent complete elliptic integral: K(k)=pi/(2*AGM(1,sqrt(1-k*k))).
// This reference does not integrate the particle or its angular equation.
double exactPeriod(double amplitude, double length, double gravity) {
    require(amplitude > 0.0 && amplitude < pi, "Period reference needs a librating pendulum");
    double arithmetic = 1.0;
    double geometric = std::cos(amplitude / 2.0);
    for (int i = 0; i < 30; ++i) {
        const double next = (arithmetic + geometric) / 2.0;
        geometric = std::sqrt(arithmetic * geometric);
        arithmetic = next;
        if (std::abs(arithmetic - geometric) < 2e-16 * arithmetic) break;
    }
    return 2.0 * pi * std::sqrt(length / gravity) / arithmetic;
}

// Cross-check the AGM result with separate Simpson quadrature, including a
// refinement check. Neither calculation calls the production integrator.
double quadraturePeriod(double amplitude, double length, double gravity, int count) {
    const double modulus = std::sin(amplitude / 2.0);
    const double h = (pi / 2.0) / count;
    double sum = 0.0;
    for (int i = 0; i <= count; ++i) {
        const double sine = std::sin(i * h);
        const double value = 1.0 / std::sqrt(1.0 - modulus * modulus * sine * sine);
        sum += (i == 0 || i == count ? 1 : (i % 2 == 0 ? 2 : 4)) * value;
    }
    return 4.0 * std::sqrt(length / gravity) * h * sum / 3.0;
}

// A separate generalized-coordinate reference for trajectory convergence.
// Check its own refinement rather than treating a small step as exact.
using AngularHistory = std::vector<std::pair<double, double>>;
AngularHistory angularReference(double amplitude, double h) {
    const int substeps = static_cast<int>(std::lround(0.005 / h));
    h = 0.005 / substeps;
    double angle = amplitude, velocity = 0.0;
    AngularHistory history{{angle, velocity}};
    const auto acceleration = [](double theta) { return -(9.81 / 1.3) * std::sin(theta); };
    for (int sample = 0; sample < 2000; ++sample) {
        for (int i = 0; i < substeps; ++i) {
            const double a1 = velocity, b1 = acceleration(angle);
            const double a2 = velocity + h * b1 / 2.0, b2 = acceleration(angle + h * a1 / 2.0);
            const double a3 = velocity + h * b2 / 2.0, b3 = acceleration(angle + h * a2 / 2.0);
            const double a4 = velocity + h * b3, b4 = acceleration(angle + h * a3);
            angle += h / 6.0 * (a1 + 2.0 * a2 + 2.0 * a3 + a4);
            velocity += h / 6.0 * (b1 + 2.0 * b2 + 2.0 * b3 + b4);
        }
        history.emplace_back(angle, velocity);
    }
    return history;
}

// Fourth-order local event interpolation from accepted x and vx. Linear
// interpolation would impose O(h^2) crossing errors on an O(h^4) integrator.
double descendingCrossing(double t0, double t1, Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d p1, Eigen::Vector3d v1) {
    const double h = t1 - t0;
    double left = 0.0, right = 1.0;
    for (int i = 0; i < 55; ++i) {
        const double s = (left + right) / 2.0;
        const double s2 = s * s, s3 = s2 * s;
        const double x = (2.0 * s3 - 3.0 * s2 + 1.0) * p0.x()
            + (s3 - 2.0 * s2 + s) * h * v0.x()
            + (-2.0 * s3 + 3.0 * s2) * p1.x() + (s3 - s2) * h * v1.x();
        if (x > 0.0) left = s;
        else right = s;
    }
    return t0 + h * (left + right) / 2.0;
}

struct Measurement {
    std::string system;
    double initial_coordinate;
    double dt;
    double phase_error = 0.0;
    double energy_error = 0.0;
    double energy_fraction = 0.0;
    double length_error = 0.0;
    double radial_velocity = 0.0;
    double reference_period = 0.0;
    double measured_period = 0.0;
    double period_error = 0.0;
    double phase_order = 0.0;
    double period_order = 0.0;
    double energy_order = 0.0;
};

void recordDiagnostics(Measurement& row, const Simulation& sim, double energy_scale) {
    const auto d = sim.diagnostics();
    row.energy_error = std::max(row.energy_error, std::abs(d.energy_change));
    row.energy_fraction = row.energy_error / energy_scale;
    row.length_error = std::max(row.length_error, d.constraints.max_position_error);
    row.radial_velocity = std::max(row.radial_velocity, d.constraints.max_velocity_error);
}

Measurement oscillator(double dt) {
    Measurement row{"oscillator", 0.7, dt};
    auto sim = makeSimulation(examples::oscillator({2.0, 8.0, 0.7, -0.3}));
    const double energy_scale = 0.5 * 2.0 * 0.3 * 0.3 + 0.5 * 8.0 * 0.7 * 0.7;
    const int count = static_cast<int>(std::lround(duration / dt));
    for (int i = 1; i <= count; ++i) {
        sim.step(i * dt - sim.time());
        const double t = sim.time();
        if (t <= 10.0 + 1e-12) {
            const double x = 0.7 * std::cos(2.0 * t) - 0.15 * std::sin(2.0 * t);
            const double v = -1.4 * std::sin(2.0 * t) - 0.3 * std::cos(2.0 * t);
            row.phase_error = std::max(row.phase_error, std::hypot(
                (sim.state().positions[0] - Eigen::Vector3d{x, 0.0, 0.0}).stableNorm(),
                (sim.state().velocities[0] - Eigen::Vector3d{v, 0.0, 0.0}).stableNorm() / 2.0));
        }
        recordDiagnostics(row, sim, energy_scale);
    }
    return row;
}

Measurement pendulum(double dt, double amplitude, const AngularHistory& reference) {
    Measurement row{"pendulum", amplitude, dt};
    row.reference_period = exactPeriod(amplitude, 1.3, 9.81);
    auto sim = makeSimulation(examples::pendulum({2.0, 1.3, 9.81, amplitude, 0.0}));
    // Excitation energy above the potential minimum, not the arbitrary total-E zero.
    const double energy_scale = 2.0 * 9.81 * 1.3 * (1.0 - std::cos(amplitude));
    std::vector<double> crossings;
    const int count = static_cast<int>(std::lround(duration / dt));
    for (int i = 1; i <= count; ++i) {
        const double previous_time = sim.time();
        const auto previous_position = sim.state().positions[0];
        const auto previous_velocity = sim.state().velocities[0];
        sim.step(i * dt - sim.time());
        const auto p = sim.state().positions[0], v = sim.state().velocities[0];
        // Compute geometric errors independently of the production diagnostics.
        require(std::abs(p.z()) < 1e-14 && std::abs(v.z()) < 1e-14, "Planar pendulum left its plane");
        const double radius = std::hypot(p.x(), p.y());
        row.length_error = std::max(row.length_error, std::abs(radius - 1.3));
        row.radial_velocity = std::max(row.radial_velocity, std::abs(p.x() * v.x() + p.y() * v.y()) / radius);
        if (sim.time() <= 10.0 + 1e-12) {
            const auto index = static_cast<std::size_t>(std::lround(sim.time() / 0.005));
            const double angle = std::atan2(p.x(), -p.y());
            const double velocity = (p.x() * v.y() - p.y() * v.x()) / (1.3 * 1.3);
            row.phase_error = std::max(row.phase_error, std::hypot(angle - reference.at(index).first,
                (velocity - reference.at(index).second) / std::sqrt(9.81 / 1.3)));
        }
        if (crossings.size() < 6 && previous_position.x() > 0.0 && p.x() <= 0.0) {
            crossings.push_back(descendingCrossing(previous_time, sim.time(), previous_position,
                previous_velocity, p, v));
        }
        recordDiagnostics(row, sim, energy_scale);
    }
    require(crossings.size() == 6, "Not enough descending crossings to measure five full periods");
    row.measured_period = (crossings.back() - crossings.front()) / 5.0;
    row.period_error = std::abs(row.measured_period - row.reference_period);
    return row;
}

double order(double coarse, double fine) {
    require(coarse > 0.0 && fine > 0.0, "Convergence error must be positive");
    return std::log2(coarse / fine);
}

void writeCsv(const std::filesystem::path& path, const std::vector<Measurement>& rows) {
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path);
    require(static_cast<bool>(output), "Cannot open numerical validation CSV");
    output.imbue(std::locale::classic());
    output << std::setprecision(17)
        << "system,initial_coordinate,dt,duration,phase_error,max_energy_change,max_energy_fraction,max_length_error,max_radial_velocity,reference_period,measured_period,period_error,phase_order,period_order,energy_order\n";
    for (const auto& row : rows) {
        output << row.system << ',' << row.initial_coordinate << ',' << row.dt << ',' << duration << ','
               << row.phase_error << ',' << row.energy_error << ',' << row.energy_fraction << ','
               << row.length_error << ',' << row.radial_velocity << ',';
        if (row.system == "pendulum") output << row.reference_period << ',' << row.measured_period << ',' << row.period_error;
        else output << ",,";
        output << ',';
        if (row.phase_order != 0.0) output << row.phase_order;
        output << ',';
        if (row.period_order != 0.0) output << row.period_order;
        output << ',';
        if (row.energy_order != 0.0) output << row.energy_order;
        output << '\n';
    }
    output.close();
    require(static_cast<bool>(output), "Cannot finish numerical validation CSV");
}

void validate(const std::vector<Measurement>& rows) {
    for (const auto& row : rows) {
        require(std::isfinite(row.phase_error) && std::isfinite(row.energy_error), "Non-finite measurement");
        require(row.length_error <= 1e-13 && row.radial_velocity <= 1e-13,
                "Pendulum position/velocity constraint exceeds 1e-13");
        if (row.phase_order != 0.0) {
            const double minimum = row.system == "oscillator" ? 3.7 : 3.4;
            require(row.phase_order >= minimum,
                    "Trajectory convergence does not demonstrate fourth order");
        }
        if (row.energy_order != 0.0) require(row.energy_order >= 4.5, "Energy drift must decrease rapidly with refinement");
        if (row.system == "pendulum" && row.period_order != 0.0) {
            require(row.period_order > 0.0, "Period error must decrease at every refinement");
        }
        if (row.dt == timesteps.back()) {
            require(row.phase_error <= (row.system == "oscillator" ? 2e-9 : 5e-7),
                    "Finest-step trajectory error exceeds acceptance criterion");
            require(row.energy_error <= (row.system == "oscillator" ? 1e-8 : 2e-6),
                    "Finest-step 50-second energy drift exceeds acceptance criterion");
            require(row.energy_fraction <= 5e-8, "Finest-step energy drift exceeds 5e-8 of excitation energy");
            if (row.system == "pendulum") {
                require(row.period_error <= 2e-7, "Finest-step period error exceeds 2e-7 s");
                require(row.period_order >= 3.0, "Finest period refinement has not reached third-order convergence");
            }
        }
    }
}
} // namespace

int main(int argc, char** argv) {
    try {
        require(argc <= 2, "Usage: physics_numerics [output.csv]");
        std::vector<Measurement> rows;
        for (double dt : timesteps) {
            auto row = oscillator(dt);
            if (!rows.empty()) {
                row.phase_order = order(rows.back().phase_error, row.phase_error);
                row.energy_order = order(rows.back().energy_error, row.energy_error);
            }
            rows.push_back(row);
        }
        for (double amplitude : {0.1, 1.1, 2.5}) {
            const double reference_period = exactPeriod(amplitude, 1.3, 9.81);
            const double quadrature = quadraturePeriod(amplitude, 1.3, 9.81, 4096);
            require(std::abs(quadrature - quadraturePeriod(amplitude, 1.3, 9.81, 2048)) < 2e-13
                    && std::abs(reference_period - quadrature) < 2e-13, "Independent period references disagree");
            const auto reference = angularReference(amplitude, 0.0000625);
            const auto coarse = angularReference(amplitude, 0.000125);
            double reference_error = 0.0;
            for (std::size_t i = 0; i < reference.size(); ++i) {
                reference_error = std::max(reference_error, std::hypot(reference[i].first - coarse[i].first,
                    (reference[i].second - coarse[i].second) / std::sqrt(9.81 / 1.3)));
            }
            require(reference_error < 1e-10, "Angular trajectory reference has not converged");
            std::cout << std::setprecision(17) << "Reference amplitude=" << amplitude
                << " period=" << reference_period << " AGM/quadrature difference=" << std::abs(reference_period - quadrature)
                << " angular refinement difference=" << reference_error << '\n';
            bool first = true;
            for (double dt : timesteps) {
                auto row = pendulum(dt, amplitude, reference);
                if (!first) {
                    row.phase_order = order(rows.back().phase_error, row.phase_error);
                    row.period_order = order(rows.back().period_error, row.period_error);
                    row.energy_order = order(rows.back().energy_error, row.energy_error);
                }
                rows.push_back(row);
                first = false;
            }
        }
        std::cout << std::setprecision(9);
        for (const auto& row : rows) {
            std::cout << row.system << " initial=" << row.initial_coordinate << " dt=" << row.dt
                << " phase_error=" << row.phase_error << " max_dE=" << row.energy_error
                << " max_dL=" << row.length_error << " max_v_radial=" << row.radial_velocity;
            if (row.system == "pendulum") std::cout << " period_error=" << row.period_error;
            std::cout << " phase_order=" << row.phase_order << " period_order=" << row.period_order
                << " energy_order=" << row.energy_order << '\n';
        }
        if (argc == 2) writeCsv(argv[1], rows);
        validate(rows);
        std::cout << "Numerical accuracy criteria passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
