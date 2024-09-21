// Particle.h
#ifndef PARTICLE_H
#define PARTICLE_H

#include "Vector3.h"

class Particle {
private:
    Vector3 m_position;
    Vector3 m_velocity;
    Vector3 m_force;        // Net force acting on the particle
    double m_mass;         // Mass of the particle

public:
    // Constructors
    Particle();
    Particle(const Vector3& position, const Vector3& velocity = Vector3(), double mass = 1.0);

    // Getters and setters
    Vector3 getPosition() const;
    void setPosition(const Vector3& position);
    Vector3 getVelocity() const;
    void setVelocity(const Vector3& velocity);
    Vector3 getForce() const;
    void setForce(const Vector3& force);
    double getMass() const;
    void setMass(double mass);

    // Energy
    double kineticEnergy() const;

    // Update the particle's state based on RK4 method
    void update(double deltaTime);

    // Overload the << operator
    friend std::ostream& operator<<(std::ostream& os, const Particle& p);
};

#endif // PARTICLE_H
