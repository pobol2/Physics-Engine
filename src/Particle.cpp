// Particle.cpp
#include "Particle.h"

// Constructors
Particle::Particle() : m_position(Vector()), m_velocity(Vector()), m_acceleration(Vector()), m_force(Vector()), m_mass(1.0) {}

Particle::Particle(const Vector& position, const Vector& velocity, const Vector& acceleration, double mass)
    : m_position(position), m_velocity(velocity), m_acceleration(acceleration), m_force(Vector()), m_mass(mass) {
    if (mass <= 0) {
        throw std::invalid_argument("Mass must be greater than zero");
    }
}

// Getters and setters
Vector Particle::getPosition() const { return m_position; }
void Particle::setPosition(const Vector& position) { m_position = position; }

Vector Particle::getVelocity() const { return m_velocity; }
void Particle::setVelocity(const Vector& velocity) { m_velocity = velocity; }

Vector Particle::getAcceleration() const { return m_acceleration; }
void Particle::setAcceleration(const Vector& acceleration) { m_acceleration = acceleration; }

Vector Particle::getForce() const { return m_force; }
void Particle::setForce(const Vector& force) { m_force = force; }

double Particle::getMass() const { return m_mass; }
void Particle::setMass(double mass) { 
    if (mass <= 0) {
        throw std::invalid_argument("Mass must be greater than zero");
    }
    m_mass = mass;
}

// Add gravitational force
void Particle::applyGravity() {
    const double gravity = 9.81; // Define gravitational acceleration (m/s²)   
    Vector gravityForce(0, -gravity * m_mass, 0); // Create a gravitational force vector
    m_force = m_force + gravityForce;  // Add gravitational force to the current force
}

// Calculate kinetic energy
double Particle::kineticEnergy() const {
    double speedSquared = m_velocity.magnitudeSquared();
    return 0.5 * m_mass * speedSquared;
}

// Calculate potential energy (using the y-axis for height)
double Particle::potentialEnergy() const {
    double height = m_position.getY();  // Assume height is the y-coordinate
    return m_mass * 9.81 * height;
}

// Calculate total mechanical energy
double Particle::totalEnergy() const {
    return kineticEnergy() + potentialEnergy();
}

// Update the particle's state based on RK4 method
void Particle::update(double deltaTime) {

    // k1
    Vector k1_p = deltaTime * m_velocity;
    Vector k1_v = deltaTime * m_force / m_mass;

    // k2
    Vector k2_p = deltaTime * (m_velocity + k1_v/2);
    Vector k2_v = k1_v;

    // k3
    Vector k3_p = deltaTime * (m_velocity + k2_v/2);
    Vector k3_v = k1_v;

    // k4
    Vector k4_p = deltaTime * (m_velocity + k3_v);
    Vector k4_v = k1_v;

    // Update position and velocity using RK4
    m_position = m_position + (k1_p + 2 * k2_p + 2 * k3_p + k4_p) / 6.0;
    m_velocity = m_velocity + (k1_v + 2 * k2_v + 2 * k3_v + k4_v) / 6.0;
}


// Print the particle's details
std::ostream& operator<<(std::ostream& os, const Particle& p) {
    os << "Position:\n" << p.m_position
       << "Velocity:\n" << p.m_velocity
       << "Acceleration:\n" << p.m_acceleration
       << "Force:\n" << p.m_force
       << "Mass: " << p.m_mass;
    return os;
}