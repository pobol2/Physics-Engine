// FixedPositionConstraint.cpp

#include "FixedPositionConstraint.h"
#include "System.h"

void FixedPositionConstraint::applyConstraint(System& system, const size_t particleIndex) const {
    system.addForce(particleIndex, -system.getSumForce(particleIndex));
}