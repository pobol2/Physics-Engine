#include "examples.hpp"

#include <stdexcept>
#include <utility>

namespace physics::examples {
namespace {
template <class Config> struct Binding {
    const char* name;
    double Config::*member;
    const char* description;
};

template <class Config>
Spec makeSpec(std::string name, std::string summary, double duration, double dt,
              std::vector<Binding<Config>> bindings, Example (*factory)(const Config&)) {
    const Config defaults{};
    std::vector<Parameter> parameters;
    for (const auto& binding : bindings) {
        parameters.push_back({binding.name, defaults.*binding.member, binding.description});
    }
    auto build = [bindings, factory](const std::map<std::string, double>& values) {
        Config config{};
        for (const auto& binding : bindings) config.*binding.member = values.at(binding.name);
        return factory(config);
    };
    return {std::move(name), std::move(summary), std::move(parameters), duration, dt, std::move(build)};
}

std::vector<Spec> makeCatalog() {
    std::vector<Spec> specs;
    specs.push_back(makeSpec<OscillatorConfig>("oscillator", "Harmonic oscillator with optional linear damping",
        10.0, 0.001, {
            {"mass", &OscillatorConfig::mass, "kg"},
            {"stiffness", &OscillatorConfig::stiffness, "N/m"},
            {"displacement", &OscillatorConfig::displacement, "initial x, m"},
            {"velocity", &OscillatorConfig::velocity, "initial vx, m/s"},
            {"damping", &OscillatorConfig::damping, "kg/s; force = -c v"}}, &oscillator));
    specs.push_back(makeSpec<PendulumConfig>("pendulum", "Simple pendulum: rigid rod from a fixed pivot",
        10.0, 0.001, {
            {"mass", &PendulumConfig::mass, "kg"},
            {"length", &PendulumConfig::length, "m"},
            {"gravity", &PendulumConfig::gravity, "m/s^2"},
            {"angle", &PendulumConfig::angle, "rad from downward vertical"},
            {"angular-velocity", &PendulumConfig::angular_velocity, "rad/s"},
            {"damping", &PendulumConfig::damping, "kg/s; force = -c v"}}, &pendulum));
    specs.push_back(makeSpec<DrivenOscillatorConfig>("driven-oscillator",
        "Damped oscillator driven by F0 cos(Omega t)", 30.0, 0.001, {
            {"mass", &DrivenOscillatorConfig::mass, "kg"},
            {"stiffness", &DrivenOscillatorConfig::stiffness, "N/m"},
            {"damping", &DrivenOscillatorConfig::damping, "kg/s"},
            {"force-amplitude", &DrivenOscillatorConfig::force_amplitude, "F0, N"},
            {"drive-frequency", &DrivenOscillatorConfig::drive_frequency, "Omega, rad/s"},
            {"displacement", &DrivenOscillatorConfig::displacement, "initial x, m"},
            {"velocity", &DrivenOscillatorConfig::velocity, "initial vx, m/s"}}, &drivenOscillator));
    specs.push_back(makeSpec<ParametricOscillatorConfig>("parametric-oscillator",
        "Oscillator with modulated stiffness (custom force)", 60.0, 0.002, {
            {"mass", &ParametricOscillatorConfig::mass, "kg"},
            {"natural-frequency", &ParametricOscillatorConfig::natural_frequency, "w0, rad/s"},
            {"modulation", &ParametricOscillatorConfig::modulation, "h, relative stiffness modulation"},
            {"modulation-frequency", &ParametricOscillatorConfig::modulation_frequency, "gamma, rad/s"},
            {"displacement", &ParametricOscillatorConfig::displacement, "initial x, m"},
            {"velocity", &ParametricOscillatorConfig::velocity, "initial vx, m/s"}}, &parametricOscillator));
    specs.push_back(makeSpec<KapitzaPendulumConfig>("kapitza-pendulum",
        "Pendulum on a vertically oscillating pivot (moving support)", 10.0, 0.0001, {
            {"mass", &KapitzaPendulumConfig::mass, "kg"},
            {"length", &KapitzaPendulumConfig::length, "m"},
            {"gravity", &KapitzaPendulumConfig::gravity, "m/s^2"},
            {"pivot-amplitude", &KapitzaPendulumConfig::pivot_amplitude, "a, m"},
            {"pivot-frequency", &KapitzaPendulumConfig::pivot_frequency, "w, rad/s"},
            {"angle", &KapitzaPendulumConfig::angle, "rad from downward vertical"},
            {"angular-velocity", &KapitzaPendulumConfig::angular_velocity, "rad/s"}}, &kapitzaPendulum));
    specs.push_back(makeSpec<DoublePendulumConfig>("double-pendulum",
        "Two rods in series (coupled constraints)", 20.0, 0.001, {
            {"mass1", &DoublePendulumConfig::mass1, "kg"},
            {"mass2", &DoublePendulumConfig::mass2, "kg"},
            {"length1", &DoublePendulumConfig::length1, "m"},
            {"length2", &DoublePendulumConfig::length2, "m"},
            {"gravity", &DoublePendulumConfig::gravity, "m/s^2"},
            {"angle1", &DoublePendulumConfig::angle1, "rad from downward vertical"},
            {"angle2", &DoublePendulumConfig::angle2, "rad from downward vertical"},
            {"angular-velocity1", &DoublePendulumConfig::angular_velocity1, "rad/s"},
            {"angular-velocity2", &DoublePendulumConfig::angular_velocity2, "rad/s"}}, &doublePendulum));
    specs.push_back(makeSpec<FoldedChainConfig>("folded-chain",
        "Chain of rigid links released folded; its free end falls faster than g", 0.6, 0.0001, {
            {"mass", &FoldedChainConfig::mass, "kg, whole chain"},
            {"length", &FoldedChainConfig::length, "m, whole chain"},
            {"links", &FoldedChainConfig::links, "number of rigid links"},
            {"gap", &FoldedChainConfig::gap, "m between the arms"},
            {"bending", &FoldedChainConfig::bending, "N/m, springs between every other mass"},
            {"gravity", &FoldedChainConfig::gravity, "m/s^2"}}, &foldedChain));
    specs.push_back(makeSpec<RotatingHoopConfig>("rotating-hoop",
        "Bead on a hoop spinning about the vertical (rheonomic constraint)", 10.0, 0.001, {
            {"mass", &RotatingHoopConfig::mass, "kg"},
            {"radius", &RotatingHoopConfig::radius, "m"},
            {"gravity", &RotatingHoopConfig::gravity, "m/s^2"},
            {"rotation-rate", &RotatingHoopConfig::rotation_rate, "Omega, rad/s"},
            {"angle", &RotatingHoopConfig::angle, "rad from downward vertical"},
            {"angular-velocity", &RotatingHoopConfig::angular_velocity, "rad/s along the hoop"}}, &rotatingHoop));
    specs.push_back(makeSpec<KeplerConfig>("kepler", "Elliptic orbit in an inverse-square central field",
        18.0, 0.001, {
            {"mass", &KeplerConfig::mass, "kg"},
            {"gravitational-parameter", &KeplerConfig::gravitational_parameter, "mu = G M, m^3/s^2"},
            {"semi-major-axis", &KeplerConfig::semi_major_axis, "m"},
            {"eccentricity", &KeplerConfig::eccentricity, "0 <= e < 1"}}, &kepler));
    specs.push_back(makeSpec<ThreeBodyConfig>("three-body", "Figure-eight choreography of three equal masses",
        12.6, 0.001, {
            {"gravitational-constant", &ThreeBodyConfig::gravitational_constant, "G"},
            {"mass", &ThreeBodyConfig::mass, "kg, each body"}}, &threeBody));
    specs.push_back(makeSpec<CyclotronConfig>("cyclotron", "Charged particle in a uniform magnetic field",
        10.0, 0.001, {
            {"mass", &CyclotronConfig::mass, "kg"},
            {"charge", &CyclotronConfig::charge, "C"},
            {"field", &CyclotronConfig::field, "B along z, T"},
            {"speed", &CyclotronConfig::speed, "initial speed across the field, m/s"},
            {"axial-velocity", &CyclotronConfig::axial_velocity, "initial speed along the field, m/s"}}, &cyclotron));
    return specs;
}
} // namespace

const std::vector<Spec>& catalog() {
    static const std::vector<Spec> specs = makeCatalog();
    return specs;
}

const Spec& find(const std::string& name) {
    for (const auto& spec : catalog()) {
        if (spec.name == name) return spec;
    }
    throw std::invalid_argument("Unknown example: " + name);
}

Example build(const Spec& spec, const std::map<std::string, double>& overrides) {
    std::map<std::string, double> values;
    for (const auto& parameter : spec.parameters) values[parameter.name] = parameter.default_value;
    for (const auto& [name, value] : overrides) {
        if (values.count(name) == 0) {
            throw std::invalid_argument("Example " + spec.name + " has no parameter " + name);
        }
        values[name] = value;
    }
    return spec.build(values);
}
} // namespace physics::examples
