// GravityForce.cpp
#include "GravityForce.h"
#include "System.h"

void GravityForce::applyForce(System& system) const {
    for (const auto& particlePtr : system.getParticles()) {
        Vector gravity(0, -m_gravity, 0);
        particlePtr->setForce(particlePtr->getForce() + gravity * particlePtr->getMass());
    }
}

double GravityForce::getPotentialEnergy(System& system) const {
    double PotentialEnergy = 0;
    for (const auto& particlePtr : system.getParticles()) {
        PotentialEnergy += particlePtr->getMass() * m_gravity * particlePtr->getPosition().getY();
    }
    return PotentialEnergy;
}