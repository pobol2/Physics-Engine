#ifndef SYSTEM_H
#define SYSTEM_H

#include <vector>
#include "Particle.h"

class System {
private:
    // TODO : Change for a custom made way to group particle 
    std::vector<Particle> m_particles;

public:
    // Add a particle to the system
    void addParticle(const Particle& particle);

    // Update all particles in the system
    void update(double deltaTime);

    // Apply gravity to all particles
    void applyGravity(const Vector& gravity);

    // Get a reference to a particle
    Particle& getParticle(size_t index);
    
    // Get the number of particles in the system
    size_t size() const;

    // Calculate total kinetic energy of the system
    double totalKineticEnergy() const;

    // Calculate total potential energy of the system
    double totalPotentialEnergy() const;

    // Calculate total mechanical energy of the system
    double totalEnergy() const;
};

#endif // SYSTEM_H
