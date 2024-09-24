// Plane.cpp
#include "Plane.h"
#include "System.h"

// Constructors
Plane::Plane() : m_normal(Vector3()), m_point(Vector3()) {}
Plane::Plane(const Vector3& normal, const Vector3& point) :  m_normal(normal), m_point(point) {}

// Getters and setters
Vector3 Plane::getNormal() const { return m_normal; }
void Plane::setNormal(const Vector3& normal) { m_normal = normal; }

Vector3 Plane::getPoint() const { return m_point; }
void Plane::setPoint(const Vector3& point) { m_point = point; }

bool Plane::checkCollision(const System& system, const size_t particleIndex) const {
    double distanceToPlane = (system.getPosition(particleIndex) - m_point).dot(m_normal);
    return distanceToPlane <= 0;
}

void Plane::resolveCollision(System& system, const size_t particleIndex) const {
    Vector3 normalVelocity = system.getVelocity(particleIndex).dot(m_normal) * m_normal;
    Vector3 tangentialVelocity = system.getVelocity(particleIndex) - normalVelocity;
    Vector3 newVelocity = tangentialVelocity - normalVelocity; // Elastic collision

    system.setVelocity(newVelocity, particleIndex);
}