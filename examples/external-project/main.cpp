// A small external application: it defines its own system with the public API,
// runs it, compares with its own analytical result, and writes its own CSV.
// None of the output or analysis below belongs in the engine.
#include "physics/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
    try {
        using namespace physics;
        const double mass = 1.0, length = 1.0, g = 9.81, amplitude = 0.05;

        // Problem-specific setup: a pendulum on a rigid rod from a fixed pivot.
        Model model;
        model.masses = {mass};
        model.gravity = {0.0, -g, 0.0};
        model.distance_constraints = {{0, Eigen::Vector3d::Zero(), length}};

        State state;
        state.positions = {{length * std::sin(amplitude), -length * std::cos(amplitude), 0.0}};
        state.velocities = {Eigen::Vector3d::Zero()};

        Simulation sim(model, state);
        AdaptiveSettings settings;
        settings.relative_tolerance = settings.absolute_tolerance = 1e-10;

        // Application-owned export and analysis.
        std::ofstream csv;
        if (argc > 1) {
            csv.open(argv[1]);
            if (!csv) {
                std::cerr << "Error: cannot open " << argv[1] << '\n';
                return 1;
            }
            csv << "time,angle,small_angle_reference\n";
        }
        const double omega = std::sqrt(g / length);
        double worst = 0.0;
        for (int i = 0; i <= 100; ++i) {
            if (i > 0) sim.advanceAdaptive(0.05 * i, settings);
            const Eigen::Vector3d& q = sim.state().positions[0];
            const double angle = std::atan2(q.x(), -q.y());
            const double reference = amplitude * std::cos(omega * sim.time());
            worst = std::max(worst, std::abs(angle - reference));
            if (csv) csv << sim.time() << ',' << angle << ',' << reference << '\n';
        }

        const Diagnostics d = sim.diagnostics();
        std::cout << "max |angle - small-angle solution| = " << worst << " rad\n"
                  << "energy-balance error = " << d.energy_balance_error << " J\n"
                  << "constraint residual = " << d.constraints.max_position_error << " m\n";
        // The small-angle solution is only approximate at finite amplitude.
        return worst < 1e-3 && std::abs(d.scaled_energy_balance_error) < 1e-8 ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
