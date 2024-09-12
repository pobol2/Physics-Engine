// Point.h
#ifndef POINT_H
#define POINT_H

#include "Vector.h"

class Point {
private:
    Vector m_position;
    Vector m_velocity;
    Vector m_acceleration;

public:
    // Constructors
    Point();
    Point(const Vector& position, const Vector& velocity = Vector(), const Vector& acceleration = Vector());

    // Getters and setters
    Vector getPosition() const;
    void setPosition(const Vector& position);
    Vector getVelocity() const;
    void setVelocity(const Vector& velocity);
    Vector getAcceleration() const;
    void setAcceleration(const Vector& acceleration);

    // Update the point's state based on a time step
    void update(double deltaTime);

    // Overload the << operator
    friend std::ostream& operator<<(std::ostream& os, const Point& p);
};

#endif // POINT_H
