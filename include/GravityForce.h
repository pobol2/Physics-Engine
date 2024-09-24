// GravityForce.h
#ifndef GRAVITY_FORCE_H
#define GRAVITY_FORCE_H

#include "Force.h"
#include "Vector3.h"

class GravityForce : public Force {
public:
    // Constructors
    GravityForce(double gravity) : m_gravity(0, -gravity, 0) {}
    
    void applyForce(System& system, const size_t particleIndex) const override;
    void applyForce(System& system) const override;

    double getPotentialEnergy(System& system) const override;  

private:
    Vector3 m_gravity;
};

#endif // GRAVITY_FORCE_H
