// DragForce.cpp
#include "DragForce.h"
#include "System.h"

void DragForce::applyForce(System& system, const size_t particleIndex) const {
    system.addForce(particleIndex, -m_cd * system.getVelocity(particleIndex).normalize());
}

void DragForce::applyForce(System& system) const {
    for (std::size_t i = 0; i < system.getNumberParticles(); ++i) {
        system.addForce(i, -m_cd * system.getVelocity(i).normalize());
    }
}