// Plane.cpp
#include "Plane.h"
#include <iostream>

// Constructors
Plane::Plane() : m_normal(Vector3()), m_point(Vector3()) {}
Plane::Plane(const Vector3& normal, const Vector3& point) :  m_normal(normal), m_point(point) {}

// Getters and setters
Vector3 Plane::getNormal() const { return m_normal; }
void Plane::setNormal(const Vector3& normal) { m_normal = normal; }

Vector3 Plane::getPoint() const { return m_point; }
void Plane::setPoint(const Vector3& point) { m_point = point; }

bool Plane::checkCollision(const Particle& particle) const {
    double distanceToPlane = (particle.getPosition() - m_point).dot(m_normal);
    return distanceToPlane <= 0;
}

void Plane::resolveCollision(Particle& particle) const {
    Vector3 normalVelocity = particle.getVelocity().dot(m_normal) * m_normal;
    Vector3 tangentialVelocity = particle.getVelocity() - normalVelocity;
    Vector3 newVelocity = tangentialVelocity - normalVelocity; // Elastic collision

    particle.setVelocity(newVelocity);
}