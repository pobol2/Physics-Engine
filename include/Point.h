// Point.h
#ifndef POINT_H
#define POINT_H

#include "Vector.h"

class Point {
private:
    Vector m_position;
    Vector m_velocity;
    Vector m_acceleration;
    Vector m_force;        // Net force acting on the point
    double m_mass;         // Mass of the point

public:
    // Constructors
    Point();
    Point(const Vector& position, const Vector& velocity = Vector(), const Vector& acceleration = Vector(), double mass = 1.0);

    // Getters and setters
    Vector getPosition() const;
    void setPosition(const Vector& position);
    Vector getVelocity() const;
    void setVelocity(const Vector& velocity);
    Vector getAcceleration() const;
    void setAcceleration(const Vector& acceleration);
    Vector getForce() const;
    void setForce(const Vector& force);
    double getMass() const;
    void setMass(double mass);

    // Apply gravity
    void applyGravity();

    // Energy
    double kineticEnergy() const;
    double potentialEnergy() const;
    double totalEnergy() const;

    // Update the point's state based on Euler method
    void update(double deltaTime);

    // Overload the << operator
    friend std::ostream& operator<<(std::ostream& os, const Point& p);
};

#endif // POINT_H
