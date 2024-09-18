// GravityForce.cpp
#include "GravityForce.h"
#include "System.h"

void GravityForce::applyForce(System& system) const {
    for (size_t i = 0; i < system.size(); ++i) {
        Particle& particle  = system.getParticle(i);
        Vector gravity(0, -m_gravity, 0);
        particle.setForce(particle.getForce() + gravity * particle.getMass());
    }
}
