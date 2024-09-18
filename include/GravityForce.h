// GravityForce.h
#ifndef GRAVITY_FORCE_H
#define GRAVITY_FORCE_H

#include "Force.h"

class GravityForce : public Force {
public:
    // Constructors
    GravityForce(double gravity) : m_gravity(gravity) {}
    
    void applyForce(System& system) const override;

private:
    double m_gravity;
};

#endif // GRAVITY_FORCE_H
