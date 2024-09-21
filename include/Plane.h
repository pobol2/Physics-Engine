// Plane.h
#ifndef PLANE_H
#define PLANE_H

#include "Vector3.h"
#include "Particle.h"

class Plane {
private:
    Vector3 m_normal; // Normal vector of the plane
    Vector3 m_point; // Point on the plane
public:

    // Constructors
    Plane();
    Plane(const Vector3& m_normal, const Vector3& m_point);

    // Getters and setters
    Vector3 getNormal() const;
    void setNormal(const Vector3& normal);
    Vector3 getPoint() const;
    void setPoint(const Vector3& point);

    // Function to check if a particle collides with the plane
    bool checkCollision(const Particle& particle) const;

    // Function to resolve the collision
    void resolveCollision(Particle& particle) const;
};

#endif // PLANE_H