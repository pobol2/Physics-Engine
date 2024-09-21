// Particle.cpp
#include "Particle.h"

// Constructors
Particle::Particle() : m_position(Vector3()), m_velocity(Vector3()), m_force(Vector3()), m_mass(1.0) {}

Particle::Particle(const Vector3& position, const Vector3& velocity, double mass)
    : m_position(position), m_velocity(velocity), m_force(Vector3()), m_mass(mass) {
    if (mass <= 0) {
        throw std::invalid_argument("Mass must be greater than zero");
    }
}

// Getters and setters
Vector3 Particle::getPosition() const { return m_position; }
void Particle::setPosition(const Vector3& position) { m_position = position; }

Vector3 Particle::getVelocity() const { return m_velocity; }
void Particle::setVelocity(const Vector3& velocity) { m_velocity = velocity; }

Vector3 Particle::getForce() const { return m_force; }
void Particle::setForce(const Vector3& force) { m_force = force; }

double Particle::getMass() const { return m_mass; }
void Particle::setMass(double mass) { 
    if (mass <= 0) {
        throw std::invalid_argument("Mass must be greater than zero");
    }
    m_mass = mass;
}

// Calculate kinetic energy
double Particle::kineticEnergy() const {
    double speedSquared = m_velocity.magnitudeSquared();
    return 0.5 * m_mass * speedSquared;
}

// Update the particle's state based on Euler method
void Particle::update(double deltaTime) {

    // Update position and velocity using Euler
    m_position = m_position + deltaTime * m_velocity;
    m_velocity = m_velocity + deltaTime * m_force / m_mass;
}

// Print the particle's details
std::ostream& operator<<(std::ostream& os, const Particle& p) {
    os << "Position:\n" << p.m_position
       << "Velocity:\n" << p.m_velocity
       << "Force:\n" << p.m_force
       << "Mass: " << p.m_mass;
    return os;
}
