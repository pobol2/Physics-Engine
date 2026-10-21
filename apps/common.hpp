#pragma once

#include "examples.hpp"
#include "physics/simulation.hpp"

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <map>
#include <string>

namespace physics::app {
enum class Integrator { RungeKutta4, Adaptive };

struct Options {
    std::string example = "oscillator";
    double dt = 0.0;       // 0: the example's suggested step
    double duration = -1.0; // negative: the example's suggested duration
    std::size_t sample_every = 1;
    std::string csv_path;
    bool help = false;
    bool list = false;
    Integrator integrator = Integrator::RungeKutta4;
    AdaptiveSettings adaptive;
    std::map<std::string, double> parameters; // example parameter overrides
};

Options parseOptions(int argc, char** argv);
std::string usage(const char* program);
std::string exampleList();

// Both front ends use the same setup, output grid, endpoint handling, sampling,
// and CSV writer. Only the viewer chooses when to request the next interval.
// With RK4, each interval of length dt is one step. With the adaptive
// integrator, dt is the output interval and steps are chosen by error control.
class Run {
public:
    explicit Run(const Options& options);
    bool finished() const { return simulation_.time() >= options_.duration; }
    void advance();
    void finishOutput();
    const Simulation& simulation() const { return simulation_; }
    const Options& options() const { return options_; }
    std::string summary() const;

private:
    void writeSample();
    Options options_;
    Simulation simulation_;
    std::ofstream csv_;
    std::uint64_t intervals_ = 0;
    std::uint64_t last_sampled_interval_ = 0;
};
} // namespace physics::app
