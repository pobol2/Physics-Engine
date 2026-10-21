#pragma once

#include <Eigen/Core>

#include <vector>

namespace physics {
using Vectors = std::vector<Eigen::Vector3d>;

// Positions and velocities of every particle; configuration lives in Model.
struct State {
    Vectors positions;
    Vectors velocities;
};

// Read-only view of a system at one evaluation point, passed to forces and
// constraints. It refers to data owned by the caller and is valid only for the
// duration of the call.
struct SystemView {
    const std::vector<double>& masses;
    const State& state;
    double time;
};
} // namespace physics
