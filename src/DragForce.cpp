// DragForce.cpp
#include "DragForce.h"
#include "System.h"

void DragForce::applyForce(System& system) const {
    for (size_t i = 0; i < system.size(); ++i) {
        Particle& particle  = system.getParticle(i);
        Vector unitVelocity = particle.getVelocity().normalize();
        particle.setForce(particle.getForce() - m_cd * unitVelocity);
    }
}