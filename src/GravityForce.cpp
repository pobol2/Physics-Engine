// GravityForce.cpp

#include <ostream>
#include "GravityForce.h"
#include "System.h"

void GravityForce::applyForce(System& system, const size_t particleIndex) const {
    system.addForce(particleIndex, system.getMass(particleIndex) * m_gravity);
}

void GravityForce::applyForce(System& system) const {
    for (std::size_t i = 0; i < system.getNumberParticles(); ++i) {
        system.addForce(i, system.getMass(i) * m_gravity);
    }
}

double GravityForce::getPotentialEnergy(System& system) const {
    double PotentialEnergy = 0;
    for (std::size_t i = 0; i < system.getNumberParticles(); ++i) {
        PotentialEnergy += system.getMass(i) * m_gravity[2] * system.getPosition(i).getY();
    }
    return PotentialEnergy;
}