// LinkConstraint.cpp

#include "LinkConstraint.h"
#include "System.h"

void LinkConstraint::applyConstraint(System& system) const {

    Vector3 d = system.getPosition(m_particle1Index) - system.getPosition(m_particle2Index);
    Vector3 dp = system.getVelocity(m_particle1Index) - system.getVelocity(m_particle2Index);
    Vector3 f1 = system.getSumForce(m_particle1Index);
    Vector3 f2 = system.getSumForce(m_particle2Index);
    double m1 = system.getMass(m_particle1Index);
    double m2 = system.getMass(m_particle2Index);
    double C = 1/2*(d.dot(d) - m_length*m_length);

    double lambda = ((m1*f2 - m2*f1).dot(d) - m1*m2*dp.dot(dp))/(d.dot(d)*(m1 + m2));

    //system.addForce(m_particle1Index, -2*lambda*d);
    system.addForce(m_particle2Index, -2*lambda*d);

    //std::cout << lambda << "  " << lambda2 << std::endl;
    //std::cout << d.magnitude() << std::endl;
}