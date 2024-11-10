// DistanceConstraint.cpp

#include "DistanceConstraint.h"
#include "System.h"

void DistanceConstraint::applyConstraint(System& system) const {

    Vector3 f = system.getSumForce(m_particleIndex);
    double m = system.getMass(m_particleIndex);
    Vector3 x = system.getPosition(m_particleIndex);
    Vector3 v = system.getVelocity(m_particleIndex);
    Vector3 c = m_attachPoint;
    Vector3 d = x - c;

    double lambda = (-f.dot(d) - m*v.dot(v))/(d.dot(d));

    system.addForce(m_particleIndex, lambda*d);
}