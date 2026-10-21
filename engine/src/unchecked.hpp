#pragma once

#include "physics/model.hpp"

// Internal entry points for callers that have already validated the model and
// time, such as the integrator. Result checks still apply.
namespace physics::unchecked {
// Requires a valid model, a valid state, and a finite time.
Dynamics evaluateDynamics(const Model& model, const State& state, double time);
Energy energy(const Model& model, const State& state, double time);
// Requires a valid model and a finite time. Still validates the state it is
// given and the state it produces, and leaves the state unchanged on failure.
void projectConstraints(const Model& model, State& state, double time);
} // namespace physics::unchecked
