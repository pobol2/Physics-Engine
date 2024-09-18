// SpringForce.cpp
#include "SpringForce.h"
#include "Particle.h"

#include <iostream>

void SpringForce::applyForce(System& system) const {
    Particle& p1 = system.getParticle(m_particle1Index); // Get first particle
    Particle& p2 = system.getParticle(m_particle2Index); // Get second particle

    // Compute displacement vector between the two particles
    Vector displacement = p2.getPosition() - p1.getPosition();
    
    // Calculate the current length of the spring
    double currentLength = displacement.magnitude();

    // Calculate the force magnitude using Hooke's law: F = -k * (L - L_0)
    double forceMagnitude = -m_stiffness * (currentLength - m_restLength);

    // Normalize the displacement vector to get the direction of the force
    Vector forceDirection = displacement.normalize();

    // Apply the spring force to both particles (equal and opposite forces)
    Vector springForce = forceDirection * forceMagnitude;

    p1.setForce(p1.getForce() - springForce);      // Apply force to particle 1
    p2.setForce(p2.getForce() + springForce);      // Apply equal and opposite force to particle 2
}
