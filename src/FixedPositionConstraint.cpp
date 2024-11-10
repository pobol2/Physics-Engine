// FixedPositionConstraint.cpp

#include "FixedPositionConstraint.h"
#include "System.h"

void FixedPositionConstraint::applyConstraint(System& system) const {
    system.addForce(m_particleIndex, -system.getSumForce(m_particleIndex));
}