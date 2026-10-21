#include "common.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace physics::app {
namespace {
double number(const std::string& text, const std::string& option) {
    std::size_t consumed = 0;
    double result;
    try { result = std::stod(text, &consumed); }
    catch (const std::exception&) { throw std::invalid_argument("Invalid number for " + option); }
    if (consumed != text.size() || !std::isfinite(result)) {
        throw std::invalid_argument("Expected a finite number for " + option);
    }
    return result;
}

bool anyExampleHas(const std::string& name) {
    for (const auto& spec : examples::catalog()) {
        for (const auto& parameter : spec.parameters) {
            if (parameter.name == name) return true;
        }
    }
    return false;
}

Simulation makeSimulation(const Options& options) {
    if (!std::isfinite(options.dt) || options.dt <= 0.0
        || !std::isfinite(options.duration) || options.duration < 0.0 || options.sample_every == 0) {
        throw std::invalid_argument("dt and sample-every must be positive; duration must be nonnegative");
    }
    auto example = examples::build(examples::find(options.example), options.parameters);
    return Simulation(std::move(example.model), std::move(example.initial_state));
}
} // namespace

Options parseOptions(int argc, char** argv) {
    Options result;
    std::map<std::string, std::string> parameter_options; // name -> option as typed, for messages
    for (int i = 1; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--help" || option == "-h") { result.help = true; return result; }
        if (option == "--list") { result.list = true; return result; }
        if (i + 1 >= argc) throw std::invalid_argument("Missing value for " + option);
        const std::string value = argv[++i];
        if (option == "--example") result.example = value;
        else if (option == "--csv") {
            if (value.empty()) throw std::invalid_argument("CSV path cannot be empty");
            result.csv_path = value;
        } else if (option == "--sample-every") {
            const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result.sample_every);
            if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || result.sample_every == 0) {
                throw std::invalid_argument("sample-every must be a positive integer");
            }
        } else if (option == "--dt") {
            result.dt = number(value, option);
            if (result.dt <= 0.0) throw std::invalid_argument("dt must be positive");
        } else if (option == "--duration") {
            result.duration = number(value, option);
            if (result.duration < 0.0) throw std::invalid_argument("duration must be nonnegative");
        } else if (option == "--integrator") {
            if (value == "rk4") result.integrator = Integrator::RungeKutta4;
            else if (value == "adaptive") result.integrator = Integrator::Adaptive;
            else throw std::invalid_argument("Integrator must be rk4 or adaptive");
        } else if (option == "--rtol") result.adaptive.relative_tolerance = number(value, option);
        else if (option == "--atol") result.adaptive.absolute_tolerance = number(value, option);
        else if (option == "--max-step") result.adaptive.max_step = number(value, option);
        else if (option.rfind("--", 0) == 0 && anyExampleHas(option.substr(2))) {
            result.parameters[option.substr(2)] = number(value, option);
            parameter_options[option.substr(2)] = option;
        } else throw std::invalid_argument("Unknown option: " + option);
    }
    const auto& spec = examples::find(result.example);
    for (const auto& [name, option] : parameter_options) {
        const bool applies = std::any_of(spec.parameters.begin(), spec.parameters.end(),
                                         [&](const examples::Parameter& p) { return p.name == name; });
        if (!applies) throw std::invalid_argument(option + " does not apply to example " + spec.name);
    }
    if (result.dt == 0.0) result.dt = spec.dt;
    if (result.duration < 0.0) result.duration = spec.duration;
    if (result.integrator == Integrator::Adaptive) {
        // Throws for invalid tolerances before any output is opened.
        Simulation probe = makeSimulation(result);
        probe.advanceAdaptive(probe.time(), result.adaptive);
    }
    // Validate before either front end opens files or a graphics window.
    (void)makeSimulation(result);
    return result;
}

std::string usage(const char* program) {
    std::ostringstream text;
    text << "Usage: " << program << " [options] [--PARAMETER VALUE ...]\n"
         << "  --example NAME                  default: oscillator (see --list)\n"
         << "  --dt SECONDS                    RK4 step, or adaptive output interval; default: per example\n"
         << "  --duration SECONDS              default: per example; zero exports the initial state\n"
         << "  --integrator rk4|adaptive       default: rk4 (fixed step); adaptive = Dormand-Prince 5(4)\n"
         << "  --rtol VALUE, --atol VALUE      adaptive tolerances; default: 1e-9 each\n"
         << "  --max-step SECONDS              adaptive step limit; default: none\n"
         << "  --csv PATH                      optional CSV output\n"
         << "  --sample-every N                CSV every N intervals; always includes endpoints\n"
         << "  --list                          list examples and their parameters\n"
         << "  --help\n"
         << "Example parameters are set by name, e.g. --example pendulum --angle 1.1\n"
         << "Examples:";
    for (const auto& spec : examples::catalog()) text << ' ' << spec.name;
    text << '\n';
    return text.str();
}

std::string exampleList() {
    std::ostringstream text;
    text.imbue(std::locale::classic());
    for (const auto& spec : examples::catalog()) {
        text << spec.name << ": " << spec.summary << " (duration " << spec.duration << " s, dt " << spec.dt << " s)\n";
        for (const auto& parameter : spec.parameters) {
            text << "  --" << std::left << std::setw(26) << parameter.name << std::setw(10) << parameter.default_value
                 << parameter.description << '\n';
        }
    }
    return text.str();
}

Run::Run(const Options& options) : options_(options), simulation_(makeSimulation(options)) {
    if (!options_.csv_path.empty()) {
        csv_.open(options_.csv_path, std::ios::out | std::ios::trunc);
        if (!csv_) throw std::runtime_error("Cannot open CSV: " + options_.csv_path);
        csv_.imbue(std::locale::classic());
        csv_ << std::setprecision(17)
             << "time,step,particle,x,y,z,vx,vy,vz,kinetic,potential,total,energy_change,scaled_energy_change,"
                "external_work,energy_balance_error,max_position_error,max_velocity_error\n";
        writeSample();
    }
}

void Run::advance() {
    if (finished()) return;
    const double next_time = std::min(options_.duration, options_.dt * static_cast<double>(intervals_ + 1));
    if (options_.integrator == Integrator::Adaptive) simulation_.advanceAdaptive(next_time, options_.adaptive);
    else simulation_.step(next_time - simulation_.time());
    ++intervals_;
    if (csv_.is_open() && (intervals_ % options_.sample_every == 0 || finished())) writeSample();
}

void Run::writeSample() {
    const auto diagnostics = simulation_.diagnostics();
    const auto& state = simulation_.state();
    for (std::size_t i = 0; i < state.positions.size(); ++i) {
        const Eigen::Vector3d& p = state.positions[i];
        const Eigen::Vector3d& v = state.velocities[i];
        csv_ << simulation_.time() << ',' << simulation_.steps() << ',' << i << ','
             << p.x() << ',' << p.y() << ',' << p.z() << ',' << v.x() << ',' << v.y() << ',' << v.z() << ','
             << diagnostics.energy.kinetic << ',' << diagnostics.energy.potential << ','
             << diagnostics.energy.total() << ',' << diagnostics.energy_change << ','
             << diagnostics.scaled_energy_change << ',' << diagnostics.external_work << ','
             << diagnostics.energy_balance_error << ',' << diagnostics.constraints.max_position_error << ','
             << diagnostics.constraints.max_velocity_error << '\n';
    }
    if (!csv_) throw std::runtime_error("Cannot write CSV: " + options_.csv_path);
    last_sampled_interval_ = intervals_;
}

void Run::finishOutput() {
    if (csv_.is_open()) {
        if (last_sampled_interval_ != intervals_) writeSample();
        csv_.close();
        if (!csv_) throw std::runtime_error("Cannot finish CSV: " + options_.csv_path);
    }
}

std::string Run::summary() const {
    const auto diagnostics = simulation_.diagnostics();
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(12) << options_.example << ": time=" << simulation_.time()
           << " steps=" << simulation_.steps() << " rejected=" << diagnostics.steps.rejected
           << " energy=" << diagnostics.energy.total()
           << " energy_change=" << diagnostics.energy_change
           << " external_work=" << diagnostics.external_work
           << " energy_balance_error=" << diagnostics.energy_balance_error
           << " max_position_error=" << diagnostics.constraints.max_position_error
           << " max_velocity_error=" << diagnostics.constraints.max_velocity_error;
    return output.str();
}
} // namespace physics::app
