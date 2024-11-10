// DistanceConstraint.h
#ifndef DISTANCE_CONSTRAINT_H
#define DISTANCE_CONSTRAINT_H

#include "Constraint.h"
#include "Vector3.h"

class DistanceConstraint : public Constraint {
public:
    // Constructors
    DistanceConstraint(size_t particleIndex, Vector3 attachPoint, double length) : m_particleIndex(particleIndex), m_attachPoint(attachPoint), m_length(length) {}
     
    void applyConstraint(System& system) const override;


private:
    size_t m_particleIndex;
    Vector3 m_attachPoint;
    double m_length;
};

#endif // DISTANCE_CONSTRAINT_H
