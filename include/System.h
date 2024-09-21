// System.h
#ifndef SYSTEM_H
#define SYSTEM_H

#include <vector>
#include <memory>
#include "Particle.h"
#include "Force.h"
#include "DragForce.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "Plane.h"

class System {
private:
    // TODO : Change for a custom made way to group particle and forces
    std::vector<std::shared_ptr<Particle>> m_particles;
    std::vector<std::shared_ptr<Plane>> m_planes;
    std::vector<std::shared_ptr<Force>> m_forces;

public:

    // Getters and setters
    void addParticle(const std::shared_ptr<Particle>& particle);
    void addParticle(const Vector3& position, const Vector3& velocity, double mass);
    const std::vector<std::shared_ptr<Particle>>& getParticles() const;
    std::shared_ptr<Particle> getParticle(size_t index) const;

    void addPlane(const std::shared_ptr<Plane>& plane);
    void addPlane(const Vector3& normal, const Vector3& point);
    const std::vector<std::shared_ptr<Plane>>& getPlanes() const;

    void addDragForce(double cd);
    void addGravityForce(double gravity);
    void addSpringForce(size_t particle1Index, size_t particle2Index, double restLength, double stiffness, double damping);
    std::vector<std::shared_ptr<Force>> getForces() const;

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
