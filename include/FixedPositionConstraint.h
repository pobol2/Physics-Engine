// FixedPositionConstraint.h
#ifndef FIXED_POSITION_CONSTRAINT_H
#define FIXED_POSITION_CONSTRAINT_H

#include "Constraint.h"
#include "Vector3.h"

class FixedPositionConstraint : public Constraint {
public:
    // Constructors
    FixedPositionConstraint(size_t particleIndex, Vector3 fixedPosition) : m_particleIndex(particleIndex), m_fixedPosition(fixedPosition) {}
    
    void applyConstraint(System& system) const override;


private:
    size_t m_particleIndex;
    Vector3 m_fixedPosition;
};

#endif // FIXED_POSITION_CONSTRAINT_H
