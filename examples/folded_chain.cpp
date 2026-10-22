#include "examples.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace physics::examples {
Example foldedChain(const FoldedChainConfig& config) {
    for (double value : {config.mass, config.length, config.links, config.gap, config.bending,
                         config.gravity}) {
        if (!std::isfinite(value)) throw std::invalid_argument("Folded chain parameters must be finite");
    }
    if (config.links < 2.0 || config.links > 1000.0 || config.links != std::floor(config.links)) {
        throw std::invalid_argument("Folded chain needs a whole number of links from 2 to 1000");
    }
    const double pi = std::acos(-1.0);
    if (config.gap <= 0.0 || pi * config.gap / 2.0 >= config.length) {
        throw std::invalid_argument("Folded chain gap must be positive and leave room for both arms");
    }
    if (config.bending < 0.0) throw std::invalid_argument("Bending stiffness must be nonnegative");
    if (config.gravity < 0.0) throw std::invalid_argument("Gravity must be nonnegative");

    const auto links = static_cast<std::size_t>(config.links);
    const double link = config.length / config.links;
    Model model;
    model.masses.assign(links, config.mass / config.links);
    model.gravity = {0.0, -config.gravity, 0.0};
    model.distance_constraints = {{0, Eigen::Vector3d::Zero(), link}};
    for (std::size_t i = 1; i < links; ++i) model.links.push_back({i - 1, i, link});
    // A spring between masses i - 1 and i + 1, relaxed when the chain is straight.
    for (std::size_t i = 1; config.bending > 0.0 && i + 1 < links; ++i) {
        model.springs.push_back({i - 1, i + 1, config.bending, 2.0 * link});
    }

    // Down the left arm, round a half circle of diameter `gap`, and up the right
    // arm to the free end beside the pivot. Masses sit at equal arc length, then
    // are projected onto the links.
    const double radius = config.gap / 2.0, arm = (config.length - pi * radius) / 2.0;
    const auto along = [&](double s) -> Eigen::Vector3d {
        if (s <= arm) return {0.0, -s, 0.0};
        if (s <= arm + pi * radius) {
            const double angle = pi + (s - arm) / radius;
            return {radius + radius * std::cos(angle), -arm + radius * std::sin(angle), 0.0};
        }
        return {config.gap, -arm + (s - arm - pi * radius), 0.0};
    };
    State state;
    for (std::size_t i = 1; i <= links; ++i) {
        state.positions.push_back(along(static_cast<double>(i) * link));
        state.velocities.push_back(Eigen::Vector3d::Zero());
    }
    projectConstraints(model, state, 0.0);
    return {model, state};
}
} // namespace physics::examples
