// Plane.cpp
#include "Plane.h"
#include <iostream>

// Constructors
Plane::Plane() : m_normal(Vector()), m_point(Vector()) {}
Plane::Plane(const Vector& normal, const Vector& point) :  m_normal(normal), m_point(point) {}

// Getters and setters
Vector Plane::getNormal() const { return m_normal; }
void Plane::setNormal(const Vector& normal) { m_normal = normal; }

Vector Plane::getPoint() const { return m_point; }
void Plane::setPoint(const Vector& point) { m_point = point; }

bool Plane::checkCollision(const Particle& particle) const {
    double distanceToPlane = (particle.getPosition() - m_point).dot(m_normal);
    return distanceToPlane <= 0;
}

void Plane::resolveCollision(Particle& particle) const {
    Vector normalVelocity = particle.getVelocity().dot(m_normal) * m_normal;
    Vector tangentialVelocity = particle.getVelocity() - normalVelocity;
    Vector newVelocity = tangentialVelocity - normalVelocity; // Elastic collision

    particle.setVelocity(newVelocity);
}