#include "System.h"

// Add a particle to the system
void System::addParticle(const Particle& particle) {
    m_particles.push_back(particle);
}

// Update all particles in the system
void System::update(double deltaTime) {
    for (Particle& particle : m_particles) {
        particle.update(deltaTime);
    }
}

// Apply gravity to all particles
void System::applyGravity(const Vector& gravity) {
    for (Particle& particle : m_particles) {
        particle.setForce(particle.getMass() * gravity);
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