// SpringForce.h
#ifndef SPRING_FORCE_H
#define SPRING_FORCE_H

#include "Force.h"
#include "Vector3.h"
#include "System.h"

class SpringForce : public Force {
public:
    // Constructor to initialize spring properties
    SpringForce(size_t particle1Index, size_t particle2Index, double restLength, double stiffness, double damping)
        : m_particle1Index(particle1Index), m_particle2Index(particle2Index), m_restLength(restLength), m_stiffness(stiffness), m_damping(damping) {}

    void applyForce(System& system) const override;

    double getPotentialEnergy(System& system) const override;

    // Method to retrieve particle indices
    void getParticles(size_t& p1Index, size_t& p2Index) const {
        p1Index = m_particle1Index;
        p2Index = m_particle2Index;
    }

private:
    size_t m_particle1Index;  // Index of the first particle
    size_t m_particle2Index;  // Index of the second particle
    double m_restLength;      // Rest length of the spring
    double m_stiffness;       // Spring constant (stiffness)
    double m_damping;      // Damping constant
};

#endif // SPRING_FORCE_H
