#include "examples.hpp"

#include <cmath>
#include <stdexcept>

namespace physics::examples {
Example kepler(const KeplerConfig& config) {
    const double mu = config.gravitational_parameter, a = config.semi_major_axis, e = config.eccentricity;
    if (!std::isfinite(mu) || mu <= 0.0 || !std::isfinite(a) || a <= 0.0 || !std::isfinite(e) || e < 0.0 || e >= 1.0) {
        throw std::invalid_argument("Kepler orbit needs mu > 0, semi-major axis > 0, and 0 <= eccentricity < 1");
    }
    Model model;
    model.masses = {config.mass};
    model.forces = {centralForce(0, Eigen::Vector3d::Zero(), inverseSquare(mu * config.mass))};
    // Periapsis on +x, moving toward +y (counterclockwise about +z).
    const double periapsis = a * (1.0 - e);
    State state{{{periapsis, 0.0, 0.0}}, {{0.0, std::sqrt(mu * (1.0 + e) / periapsis), 0.0}}};
    validateModel(model);
    validateState(model, state);
    return {model, state};
}

Example threeBody(const ThreeBodyConfig& config) {
    const double g = config.gravitational_constant, m = config.mass;
    if (!std::isfinite(g) || g <= 0.0 || !std::isfinite(m) || m <= 0.0) {
        throw std::invalid_argument("Three-body constant and mass must be finite and positive");
    }
    // Initial conditions for G = m = 1 (Chenciner & Montgomery 2000, digits from
    // Simo). Velocities scale with sqrt(G m) at fixed positions.
    const Eigen::Vector3d x1{0.97000436, -0.24308753, 0.0};
    const Eigen::Vector3d v3 = Eigen::Vector3d{-0.93240737, -0.86473146, 0.0} * std::sqrt(g * m);
    Model model;
    model.masses = {m, m, m};
    model.forces = {gravitation(g)};
    State state{{x1, -x1, Eigen::Vector3d::Zero()}, {-v3 / 2.0, -v3 / 2.0, v3}};
    validateModel(model);
    validateState(model, state);
    return {model, state};
}

Example cyclotron(const CyclotronConfig& config) {
    for (double value : {config.charge, config.field, config.speed, config.axial_velocity}) {
        if (!std::isfinite(value)) throw std::invalid_argument("Cyclotron parameters must be finite");
    }
    Model model;
    model.masses = {config.mass};
    model.forces = {lorentzForce(0, config.charge, {}, uniformField({0.0, 0.0, config.field}))};
    State state{{Eigen::Vector3d::Zero()}, {{config.speed, 0.0, config.axial_velocity}}};
    validateModel(model);
    validateState(model, state);
    return {model, state};
}
} // namespace physics::examples
