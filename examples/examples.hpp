#pragma once

#include "physics/model.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>

// Ready-made systems built only from the public engine API. They double as
// usage examples and as validation targets with independent references.
// Angles are in radians from the downward vertical (-y), positive toward +x.
namespace physics::examples {
struct Example {
    Model model;
    State initial_state;
};

struct OscillatorConfig {
    double mass = 1.0;
    double stiffness = 4.0;
    double displacement = 1.0;
    double velocity = 0.0;
    double damping = 0.0; // kg/s
};

struct PendulumConfig {
    double mass = 1.0;
    double length = 1.0;
    double gravity = 9.81;
    double angle = 0.5;
    double angular_velocity = 0.0;
    double damping = 0.0; // kg/s, applied to Cartesian bob velocity
};

// m x'' + c x' + k x = F0 cos(Omega t): time-dependent external force.
struct DrivenOscillatorConfig {
    double mass = 1.0;
    double stiffness = 4.0;
    double damping = 0.2;
    double force_amplitude = 1.0;
    double drive_frequency = 1.8; // rad/s
    double displacement = 0.0;
    double velocity = 0.0;
};

// x'' + w0^2 (1 + h cos(gamma t)) x = 0 through a custom time-dependent potential.
struct ParametricOscillatorConfig {
    double mass = 1.0;
    double natural_frequency = 1.0; // w0, rad/s
    double modulation = 0.1;        // h, |h| < 1
    double modulation_frequency = 2.0; // gamma, rad/s; parametric resonance near 2 w0
    double displacement = 0.01;
    double velocity = 0.0;
};

// Pendulum whose pivot oscillates vertically: y_pivot = a cos(w t).
struct KapitzaPendulumConfig {
    double mass = 1.0;
    double length = 1.0;
    double gravity = 9.81;
    double pivot_amplitude = 0.05;
    double pivot_frequency = 200.0; // rad/s; inverted state is stable if (a w)^2 > 2 g l
    double angle = 2.9;
    double angular_velocity = 0.0;
};

// Two rods in series: a distance constraint and a link solved together.
struct DoublePendulumConfig {
    double mass1 = 1.0;
    double mass2 = 1.0;
    double length1 = 1.0;
    double length2 = 1.0;
    double gravity = 9.81;
    double angle1 = 1.2;
    double angle2 = 0.6;
    double angular_velocity1 = 0.0;
    double angular_velocity2 = 0.0;
};

// Chain of equal rigid links hanging from a fixed pivot, released at rest folded
// in a narrow U with its free end beside the pivot. The free end falls faster
// than g. Optional springs between every other mass resist sharp kinks.
struct FoldedChainConfig {
    double mass = 1.0;    // whole chain, kg
    double length = 2.0;  // whole chain, m
    double links = 30.0;  // whole number
    double gap = 0.14;    // between the arms, m
    double bending = 0.0; // N/m
    double gravity = 9.81;
};

// Bead on a circular hoop spinning at a constant rate about the vertical axis:
// a fixed-radius constraint plus a rotating-plane constraint.
struct RotatingHoopConfig {
    double mass = 1.0;
    double radius = 1.0;
    double gravity = 9.81;
    double rotation_rate = 5.0; // rad/s
    double angle = 0.3;
    double angular_velocity = 0.0;
};

// Particle in an inverse-square field of a fixed center, released at periapsis.
struct KeplerConfig {
    double mass = 1.0;
    double gravitational_parameter = 1.0; // mu = G M
    double semi_major_axis = 1.0;
    double eccentricity = 0.6;
};

// Chenciner-Montgomery figure-eight choreography of three equal masses.
struct ThreeBodyConfig {
    double gravitational_constant = 1.0;
    double mass = 1.0;
};

// Charge in a uniform magnetic field along z: helical motion.
struct CyclotronConfig {
    double mass = 1.0;
    double charge = 1.0;
    double field = 2.0;          // tesla
    double speed = 1.0;          // perpendicular to the field
    double axial_velocity = 0.2; // along the field
};

Example oscillator(const OscillatorConfig& config = {});
Example pendulum(const PendulumConfig& config = {});
Example drivenOscillator(const DrivenOscillatorConfig& config = {});
Example parametricOscillator(const ParametricOscillatorConfig& config = {});
Example kapitzaPendulum(const KapitzaPendulumConfig& config = {});
Example doublePendulum(const DoublePendulumConfig& config = {});
Example foldedChain(const FoldedChainConfig& config = {});
Example rotatingHoop(const RotatingHoopConfig& config = {});
Example kepler(const KeplerConfig& config = {});
Example threeBody(const ThreeBodyConfig& config = {});
Example cyclotron(const CyclotronConfig& config = {});

// Figure-eight period for G = m = 1 to the precision of the initial conditions;
// it scales as 1/sqrt(G m).
constexpr double figure_eight_period = 6.32591398;

// Name-based access for front ends. Parameters are numbers with defaults taken
// from the typed config; build() receives a complete name-to-value map.
struct Parameter {
    std::string name;
    double default_value;
    std::string description;
};

struct Spec {
    std::string name;
    std::string summary;
    std::vector<Parameter> parameters;
    double duration; // suggested run length, s
    double dt;       // suggested fixed step, s
    std::function<Example(const std::map<std::string, double>&)> build;
};

const std::vector<Spec>& catalog();
// Throws std::invalid_argument for an unknown name.
const Spec& find(const std::string& name);
// Fills defaults; throws std::invalid_argument for a parameter the example lacks.
Example build(const Spec& spec, const std::map<std::string, double>& overrides = {});
} // namespace physics::examples
