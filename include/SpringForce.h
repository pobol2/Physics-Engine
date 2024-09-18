// SpringForce.h
#ifndef SPRING_FORCE_H
#define SPRING_FORCE_H

#include "Force.h"
#include "Vector.h"
#include "System.h"

class SpringForce : public Force {
public:
    // Constructor to initialize spring properties
    SpringForce(size_t particle1Index, size_t particle2Index, double restLength, double stiffness)
        : m_particle1Index(particle1Index), m_particle2Index(particle2Index), m_restLength(restLength), m_stiffness(stiffness) {}

    void applyForce(System& system) const override;

    double getPotentialEnergy(System& system) const override;

private:
    size_t m_particle1Index;  // Index of the first particle
    size_t m_particle2Index;  // Index of the second particle
    double m_restLength;      // Rest length of the spring
    double m_stiffness;       // Spring constant (stiffness)
};

#endif // SPRING_FORCE_H
