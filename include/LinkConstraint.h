// LinkConstraint.h
#ifndef LINK_CONSTRAINT_H
#define LINK_CONSTRAINT_H

#include "Constraint.h"
#include "Vector3.h"

class LinkConstraint : public Constraint {
public:
    // Constructors
    LinkConstraint(size_t particle1Index, size_t particle2Index, double length) : m_particle1Index(particle1Index), m_particle2Index(particle2Index), m_length(length) {}
    
    void applyConstraint(System& system) const override;


private:
    size_t m_particle1Index;
    size_t m_particle2Index;
    double m_length;
};

#endif // LINK_CONSTRAINT_H
