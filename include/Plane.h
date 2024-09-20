// Plane.h
#ifndef PLANE_H
#define PLANE_H

#include "Vector.h"
#include "Particle.h"

class Plane {
private:
    Vector m_normal; // Normal vector of the plane
    Vector m_point; // Point on the plane
public:

    // Constructors
    Plane();
    Plane(const Vector& m_normal, const Vector& m_point);

    // Getters and setters
    Vector getNormal() const;
    void setNormal(const Vector& normal);
    Vector getPoint() const;
    void setPoint(const Vector& point);

    // Function to check if a particle collides with the plane
    bool checkCollision(const Particle& particle) const;

    // Function to resolve the collision
    void resolveCollision(Particle& particle) const;
};

#endif // PLANE_H