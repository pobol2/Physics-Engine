// System.cpp
#include "System.h"

// ---------------------- Particles ---------------------- //

// Method to add a shared_ptr to the list of particles
void System::addParticle(const std::shared_ptr<Particle>& particle) {
    m_particles.push_back(particle);
}

// Method to add a particles to the list using position, velocity, and mass
void System::addParticle(const Vector& position, const Vector& velocity, double mass) {
    addParticle(std::make_shared<Particle>(position, velocity, mass));
}

// Method to retrieve the list of shared_ptrs to particles
const std::vector<std::shared_ptr<Particle>>& System::getParticles() const {
    return m_particles;
}

// Method to get a Particle by index
std::shared_ptr<Particle> System::getParticle(size_t index) const {
    if (index >= m_particles.size()) {
        return nullptr; // Or throw an exception if preferred
    }
    return m_particles[index]; // Return the shared_ptr to the Particle
}

// ---------------------- Planes ---------------------- //

// Method to add a shared_ptr to the list of planes
void System::addPlane(const std::shared_ptr<Plane>& plane) {
    m_planes.push_back(plane);
}

// Method to add a Plane to the list using normal and point
void System::addPlane(const Vector& normal, const Vector& point) {
    addPlane(std::make_shared<Plane>(normal, point));
}

// Method to retrieve the list of shared_ptrs to planes
const std::vector<std::shared_ptr<Plane>>& System::getPlanes() const {
    return m_planes;
}

// ---------------------- Forces ---------------------- //

// Method to add a DragForce
void System::addDragForce(double cd) {
    m_forces.push_back(std::make_shared<DragForce>(cd));
}

// Method to add a GravityForce
void System::addGravityForce(double gravity) {
    m_forces.push_back(std::make_shared<GravityForce>(gravity));
}

// Method to add a SpringForce
void System::addSpringForce(size_t particle1Index, size_t particle2Index, double restLength, double stiffness, double damping) {
    m_forces.push_back(std::make_shared<SpringForce>(particle1Index, particle2Index, restLength, stiffness, damping));
}

 // Method to get a force by index
std::vector<std::shared_ptr<Force>> System::getForces() const {
    return m_forces;
}

// ---------------------- Simulation ---------------------- //

// Method to apply all forces to the system
void System::applyForces() {
    // First, reset the forces on all particles
    for (const auto& particlePtr : m_particles) {
        particlePtr->setForce(Vector(0, 0, 0));  // Reset force to zero
    }

    // Then, apply each force in the force list
    for (const auto& forcePtr : m_forces) {
        forcePtr->applyForce(*this); // Apply each force to the system
    }
}

// Check and resolve collisions with planes after updating particles
void System::handleCollisions(Particle& particle) {
    for (const auto& planePtr : m_planes) {
        if (planePtr->checkCollision(particle)) {
            planePtr->resolveCollision(particle);
        }
    }
}

// Update all particles in the system
void System::update(double deltaTime) {
    applyForces();
    for (const auto& particlePtr : m_particles) {
        particlePtr->update(deltaTime);
        handleCollisions(*particlePtr); // Handle collisions with planes
    }
}

// ---------------------- Energy ---------------------- //

// Calculate total kinetic energy of the system
double System::totalKineticEnergy() const {
    double totalKE = 0.0;
    for (const auto& particlePtr : m_particles) {
        totalKE += particlePtr->kineticEnergy();
    }
    return totalKE;
}

// Calculate total potential energy of the system
double System::totalPotentialEnergy() {
    double totalPE = 0.0;
    for (const auto& forcePtr : m_forces) {
        totalPE += forcePtr->getPotentialEnergy(*this);
    }
    return totalPE;
}

// Calculate total mechanical energy of the system
double System::totalEnergy() {
    return totalKineticEnergy() + totalPotentialEnergy();
}
