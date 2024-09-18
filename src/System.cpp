// System.cpp
#include "System.h"

// Add a particle to the system
void System::addParticle(const Particle& particle) {
    m_particles.push_back(particle);
}

// Update all particles in the system
void System::update(double deltaTime) {
    applyForces();
    for (Particle& particle : m_particles) {
        particle.update(deltaTime);
    }
}

void System::addForce(const Force& force) {
    m_forces.push_back(&force);
}

void System::applyForces() {
    // First, reset the forces on all particles
    for (Particle& particle : m_particles) {
        particle.setForce(Vector(0, 0, 0));  // Reset force to zero
    }

    // Then, apply each force in the force list
    for (const auto& force : m_forces) {
        force->applyForce(*this);  // Apply forces to the system
    }
}

// Get a reference to a particle
Particle& System::getParticle(size_t index) {
    if (index < System::size()) {
        return m_particles[index];
    } else {
        throw std::out_of_range("Index out of range");
    }
}

// Get the number of particles in the system
size_t System::size() const {
    return m_particles.size();
}

// Calculate total kinetic energy of the system
double System::totalKineticEnergy() const {
    double totalKE = 0.0;
    for (const Particle& particle : m_particles) {
        totalKE += particle.kineticEnergy();
    }
    return totalKE;
}

// Calculate total potential energy of the system
double System::totalPotentialEnergy() const {
    double totalPE = 0.0;
    for (const Particle& particle : m_particles) {
        totalPE += particle.potentialEnergy();
    }
    return totalPE;
}

// Calculate total mechanical energy of the system
double System::totalEnergy() const {
    return totalKineticEnergy() + totalPotentialEnergy();
}