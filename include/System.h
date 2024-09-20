// System.h
#ifndef SYSTEM_H
#define SYSTEM_H

#include <vector>
#include "Particle.h"
#include "Force.h"
#include "Plane.h"

class System {
private:
    // TODO : Change for a custom made way to group particle and forces
    std::vector<Particle> m_particles;
    std::vector<const Force*> m_forces;
    std::vector<Plane> m_planes;

public:

    // Getters and setters
    Particle& getParticle(size_t index);
    void addParticle(const Particle& particle);
    const std::vector<const Force*>& getForces() const;
    void addForce(const Force& force);
    const std::vector<Plane>& getPlanes() const;
    void addPlane(const Plane& Plane);

    // Get the number of particles in the system
    size_t size() const;
    
    // Add the force to each particle
    void applyForces();

    // Check and resolve collisions with planes after updating particles
    void handleCollisions(Particle& particle);

    // Update all particles in the system
    void update(double deltaTime); 
    
    // Calculate total kinetic energy of the system
    double totalKineticEnergy() const;

    // Calculate total potential energy of the system
    double totalPotentialEnergy();

    // Calculate total mechanical energy of the system
    double totalEnergy();
};

#endif // SYSTEM_H
