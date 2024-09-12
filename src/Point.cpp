// Point.cpp
#include "Point.h"

// Constructors
Point::Point() : m_position(Vector()), m_velocity(Vector()), m_acceleration(Vector()) {}
Point::Point(const Vector& position, const Vector& velocity, const Vector& acceleration)
    : m_position(position), m_velocity(velocity), m_acceleration(acceleration) {}

 // Getters and setters
Vector Point::getPosition() const { return m_position; }
void Point::setPosition(const Vector& position) { m_position = position; }

Vector Point::getVelocity() const { return m_velocity; }
void Point::setVelocity(const Vector& velocity) { m_velocity = velocity; }

Vector Point::getAcceleration() const { return m_acceleration; }
void Point::setAcceleration(const Vector& acceleration) { m_acceleration = acceleration; }

// Update the point's state based on Euler method
void Point::update(double deltaTime) {
    m_velocity = m_velocity + m_acceleration * deltaTime; // Update velocity
    m_position = m_position + m_velocity * deltaTime; // Update position
}

// Print the point's details
std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << "Position:\n" << p.m_position
       << "Velocity:\n" << p.m_velocity
       << "Acceleration:\n" << p.m_acceleration;
    return os;
}
