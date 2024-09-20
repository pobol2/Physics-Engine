// DragForce.cpp
#include "DragForce.h"
#include "System.h"

void DragForce::applyForce(System& system) const {
    for (const auto& particlePtr : system.getParticles()) {
        Vector unitVelocity = particlePtr->getVelocity().normalize();
        particlePtr->setForce(particlePtr->getForce() - m_cd * unitVelocity);
    }
}