// Point.cpp
#include "Point.h"

// Constructors
Point::Point() : m_position(Vector()), m_velocity(Vector()), m_acceleration(Vector()), m_force(Vector()), m_mass(1.0) {}

Point::Point(const Vector& position, const Vector& velocity, const Vector& acceleration, double mass)
    : m_position(position), m_velocity(velocity), m_acceleration(acceleration), m_force(Vector()), m_mass(mass) {
    if (mass <= 0) {
        throw std::invalid_argument("Mass must be greater than zero");
    }
}

// Getters and setters
Vector Point::getPosition() const { return m_position; }
void Point::setPosition(const Vector& position) { m_position = position; }

Vector Point::getVelocity() const { return m_velocity; }
void Point::setVelocity(const Vector& velocity) { m_velocity = velocity; }

Vector Point::getAcceleration() const { return m_acceleration; }
void Point::setAcceleration(const Vector& acceleration) { m_acceleration = acceleration; }

Vector Point::getForce() const { return m_force; }
void Point::setForce(const Vector& force) { m_force = force; }

double Point::getMass() const { return m_mass; }
void Point::setMass(double mass) { 
    if (mass <= 0) {
        throw std::invalid_argument("Mass must be greater than zero");
    }
    m_mass = mass;
}

// Add gravitational force
void Point::applyGravity() {
    const double gravity = 9.81; // Define gravitational acceleration (m/s²)   
    Vector gravityForce(0, -gravity * m_mass, 0); // Create a gravitational force vector
    m_force = m_force + gravityForce;  // Add gravitational force to the current force
}

// Calculate kinetic energy
double Point::kineticEnergy() const {
    double speedSquared = m_velocity.magnitudeSquared();
    return 0.5 * m_mass * speedSquared;
}

// Calculate potential energy (using the y-axis for height)
double Point::potentialEnergy() const {
    double height = m_position.getY();  // Assume height is the y-coordinate
    return m_mass * 9.81 * height;
}

// Calculate total mechanical energy
double Point::totalEnergy() const {
    return kineticEnergy() + potentialEnergy();
}

// Update the point's state based on Euler method
void Point::update(double deltaTime) {
    if (m_mass <= 0) {
        throw std::logic_error("Mass must be greater than zero before updating.");
    }

    // Calculate acceleration from force and mass (Newton’s Second Law)
    m_acceleration = m_force * (1.0 / m_mass);

    // Update velocity and position
    m_velocity = m_velocity + m_acceleration * deltaTime;
    m_position = m_position + m_velocity * deltaTime;
}

// Print the point's details
std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << "Position:\n" << p.m_position
       << "Velocity:\n" << p.m_velocity
       << "Acceleration:\n" << p.m_acceleration
       << "Force:\n" << p.m_force
       << "Mass: " << p.m_mass;
    return os;
}