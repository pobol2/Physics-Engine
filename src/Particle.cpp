// Particle.cpp
#include "Particle.h"

// Constructors
Particle::Particle() : m_position(Vector()), m_velocity(Vector()), m_force(Vector()), m_mass(1.0) {}

Particle::Particle(const Vector& position, const Vector& velocity, double mass)
    : m_position(position), m_velocity(velocity), m_force(Vector()), m_mass(mass) {
    if (mass <= 0) {
        throw std::invalid_argument("Mass must be greater than zero");
    }
}

// Getters and setters
Vector Particle::getPosition() const { return m_position; }
void Particle::setPosition(const Vector& position) { m_position = position; }

Vector Particle::getVelocity() const { return m_velocity; }
void Particle::setVelocity(const Vector& velocity) { m_velocity = velocity; }

Vector Particle::getForce() const { return m_force; }
void Particle::setForce(const Vector& force) { m_force = force; }

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

// Update the particle's state based on RK4 method
void Particle::update(double deltaTime) {

    // k1
    Vector k1_p = m_velocity;
    Vector k1_v = m_force / m_mass;

    // k2
    Vector k2_p = m_velocity + deltaTime * k1_v / 2;
    Vector k2_v = m_force / m_mass;

    // k3
    Vector k3_p = m_velocity + deltaTime * k2_v/2;
    Vector k3_v = m_force / m_mass;

    // k4
    Vector k4_p = m_velocity + deltaTime * k3_v;
    Vector k4_v = m_force / m_mass;


    // Update position and velocity using RK4
    m_position = m_position + deltaTime * (k1_p + 2 * k2_p + 2 * k3_p + k4_p) / 6.0;
    m_velocity = m_velocity + deltaTime * (k1_v + 2 * k2_v + 2 * k3_v + k4_v) / 6.0;
}


// Print the particle's details
std::ostream& operator<<(std::ostream& os, const Particle& p) {
    os << "Position:\n" << p.m_position
       << "Velocity:\n" << p.m_velocity
       << "Force:\n" << p.m_force
       << "Mass: " << p.m_mass;
    return os;
}
