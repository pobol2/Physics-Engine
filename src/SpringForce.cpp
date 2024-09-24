// SpringForce.cpp
#include "SpringForce.h"
#include "System.h"

void SpringForce::applyForce(System& system, const size_t particleIndex) const {
    // TODO
}

void SpringForce::applyForce(System& system) const {
    // Compute displacement vector between the two particles
    Vector3 displacement = system.getPosition(m_particle1Index) - system.getPosition(m_particle2Index);
    
    // Calculate the current length of the spring
    double currentLength = displacement.magnitude();

    // Calculate the force magnitude using Hooke's law: F = -k * (L - L_0)
    double forceMagnitude = -m_stiffness * (currentLength - m_restLength);

    // Normalize the displacement vector to get the direction of the force
    Vector3 forceDirection = displacement.normalize();

    // Apply the spring force to both particles (equal and opposite forces)
    Vector3 springForce = forceDirection * forceMagnitude;

    // Calculate relative velocity
    Vector3 relativeVelocity = system.getVelocity(m_particle1Index) - system.getVelocity(m_particle2Index);

    // Calculate damping force: F_damping = -c * (v2 - v1)
    double dampingMagnitude = -m_damping * relativeVelocity.dot(forceDirection);
    Vector3 dampingForce = forceDirection * dampingMagnitude;

    // Total force
    Vector3 totalForce = springForce + dampingForce;

    system.addForce(m_particle1Index, totalForce);  // Apply force to particle 1
    system.addForce(m_particle2Index, -totalForce); // Apply equal and opposite force to particle 2
}

double SpringForce::getPotentialEnergy(System& system) const {
    // Compute displacement vector between the two particles
    Vector3 displacement = system.getPosition(m_particle1Index) - system.getPosition(m_particle2Index);
    
    // Calculate the current length of the spring
    double currentLength = displacement.magnitude();

    double potentialEnergy = 0.5 * m_stiffness * currentLength * currentLength;

    return potentialEnergy;
}