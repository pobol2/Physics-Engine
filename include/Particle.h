// Particle.h
#ifndef PARTICLE_H
#define PARTICLE_H

#include "Vector.h"

class Particle {
private:
    Vector m_position;
    Vector m_velocity;
    Vector m_acceleration;
    Vector m_force;        // Net force acting on the particle
    double m_mass;         // Mass of the particle

public:
    // Constructors
    Particle();
    Particle(const Vector& position, const Vector& velocity = Vector(), const Vector& acceleration = Vector(), double mass = 1.0);

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

    // Energy
    double kineticEnergy() const;

    // Update the particle's state based on Euler method
    void update(double deltaTime);

    // Overload the << operator
    friend std::ostream& operator<<(std::ostream& os, const Particle& p);
};

#endif // PARTICLE_H
