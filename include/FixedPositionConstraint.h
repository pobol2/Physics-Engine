// FixedPositionConstraint.h
#ifndef FIXED_POSITION_CONSTRAINT_H
#define FIXED_POSITION_CONSTRAINT_H

#include "Constraint.h"
#include "Vector3.h"

class FixedPositionConstraint : public Constraint {
public:
    // Constructors
    FixedPositionConstraint(Vector3 fixedPosition) : m_fixedPosition(fixedPosition) {}
    
    void applyConstraint(System& system, const size_t particleIndex) const override;


private:
    Vector3 m_fixedPosition;
};

#endif // FIXED_POSITION_CONSTRAINT_H
