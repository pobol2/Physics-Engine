// System.h
#ifndef SYSTEM_H
#define SYSTEM_H

#include <iostream>
#include <vector>
#include <memory>

#include "VectorN.h"
#include "Vector3.h"
#include "Force.h"
#include "GravityForce.h"
#include "DragForce.h"
#include "SpringForce.h"
#include "Plane.h"
#include "Constraint.h"
#include "FixedPositionConstraint.h"
#include "LinkConstraint.h"
#include "DistanceConstraint.h"

class System {
private:

    // Particles
    size_t numberParticles = 0;
    VectorN<Vector3> m_positions;
    VectorN<Vector3> m_velocities;
    VectorN<Vector3> m_sumforces;
    VectorN<double> m_masses;
    VectorN<double> m_inverseMasses;

    // Forces
    std::vector<std::shared_ptr<Force>> m_forces;
    std::vector<std::shared_ptr<Force>> m_forcesUnary;
    std::vector<std::shared_ptr<Force>> m_forcesNary;

    // Planes
    std::vector<std::shared_ptr<Plane>> m_planes;

    // Constraints
    std::vector<std::shared_ptr<Constraint>> m_constraints;

public:

    // ---------------------- Particles ---------------------- //

    // Method to add a particles to the system using position, velocity, and mass
    void addParticle(const Vector3& position, const Vector3& velocity, double mass) {
        m_positions.append(position);
        m_velocities.append(velocity);
        m_sumforces.append(Vector3(0, 0, 0));
        m_masses.append(mass);
        m_inverseMasses.append(1/mass);

        numberParticles++;
    }

    // Getters
    VectorN<Vector3> getPositions() const { return m_positions; }
    VectorN<Vector3> getVelocities() const { return m_velocities; }
    VectorN<double> getMasses() const { return m_masses; }
    VectorN<double> getInverseMasses() const { return m_inverseMasses; }
    Vector3 getPosition(const size_t particleIndex) const { return m_positions[particleIndex]; };
    Vector3 getVelocity(const size_t particleIndex) const { return m_velocities[particleIndex]; };
    double getInverseMass(const size_t particleIndex) const { return m_inverseMasses[particleIndex]; }
    double getMass(const size_t particleIndex) const { return m_masses[particleIndex]; }
    size_t getNumberParticles() const { return numberParticles; }

    // Setters
    void setVelocity(Vector3 newVelocity, const size_t particleIndex) { m_velocities[particleIndex] = newVelocity; }


    // ---------------------- Planes ---------------------- //

    // Method to add a Plane to the list using normal and point
    void addPlane(const Vector3& normal, const Vector3& point) {
        m_planes.push_back(std::make_shared<Plane>(normal, point));
    }

    // Getters
    std::vector<std::shared_ptr<Plane>> getPlanes() const { return m_planes; }

    // ---------------------- Forces ---------------------- //

    // Method to add a DragForce
    void addDragForce(double cd) {
        m_forcesUnary.push_back(std::make_shared<DragForce>(cd));
        m_forces.push_back(std::make_shared<DragForce>(cd));
    }

    // Method to add a GravityForce
    void addGravityForce(double gravity) {
        m_forcesUnary.push_back(std::make_shared<GravityForce>(gravity));
        m_forces.push_back(std::make_shared<GravityForce>(gravity));
    }

    // Method to add a SpringForce
    void addSpringForce(size_t particle1Index, size_t particle2Index, double restLength, double stiffness, double damping) {
        m_forcesNary.push_back(std::make_shared<SpringForce>(particle1Index, particle2Index, restLength, stiffness, damping));
        m_forces.push_back(std::make_shared<SpringForce>(particle1Index, particle2Index, restLength, stiffness, damping));
    }

    // Method to increment sum forces for a specific particle
    void addForce(size_t particleIndex, const Vector3& force) { m_sumforces[particleIndex] += force; }

    // Getters
    VectorN<Vector3> getSumForces() const { return m_sumforces; }
    Vector3 getSumForce(const size_t particleIndex) const { return m_sumforces[particleIndex]; };
    std::vector<std::shared_ptr<Force>> getForces() const { return m_forces; }

    // ---------------------- Constraints ---------------------- //

    // Method to add a fixed position contraint
    void addFixedPositionConstraint(size_t particleIndex, Vector3 fixedPosition) {
        m_constraints.push_back(std::make_shared<FixedPositionConstraint>(particleIndex, fixedPosition));
    }

    // Method to add a fixed position contraint
    void addLinkConstraint(size_t particle1Index, size_t particle2Index, double length) {
        m_constraints.push_back(std::make_shared<LinkConstraint>(particle1Index, particle2Index, length));
    }

    // Method to add a distance position contraint
    void addDistanceConstraint(size_t particleIndex, Vector3 attachPoint, double length) {
        m_constraints.push_back(std::make_shared<DistanceConstraint>(particleIndex, attachPoint, length));
    }

    // ---------------------- Simulation ---------------------- //

    // Update the state of the system using Euler method
    void update(double deltaTime) {

        // Apply forces
        for (const std::shared_ptr<Force>& forcesPtr : m_forces) { 
            forcesPtr->applyForce(*this);
        }

        // Apply contraints
        for (const std::shared_ptr<Constraint>& constraintPtr : m_constraints) {
            constraintPtr->applyConstraint(*this);
        }
        
        for (std::size_t i = 0; i < numberParticles; ++i) {

            // Update position and velocity
            m_positions[i] += m_velocities[i] * deltaTime;
            m_velocities[i] += m_inverseMasses[i] * m_sumforces[i] * deltaTime;

            // Handle collisions
            for (const std::shared_ptr<Plane>& planePtr : m_planes) {
                if (planePtr->checkCollision(*this, i)) {
                    planePtr->resolveCollision(*this, i);
                }
            }

            m_sumforces[i] = Vector3(0, 0, 0); // Reset the forces on all particles
        }
    }

    // ---------------------- Energy ---------------------- //

    // Calculate total kinetic energy of the system
    double totalKineticEnergy() const {
        double totalKE = 0.0;
        for (std::size_t i = 0; i < numberParticles; ++i) {
            totalKE += 0.5 * m_velocities[i].magnitudeSquared();
        }
        return totalKE;
    }

    // Calculate total potential energy of the system
    double totalPotentialEnergy() {
        double totalPE = 0.0;
        for (const auto& forcePtr : m_forces) {
            totalPE += forcePtr->getPotentialEnergy(*this);
        }
        return totalPE;
    }

    // Calculate total mechanical energy of the system
    double totalEnergy() {
        return totalKineticEnergy() + totalPotentialEnergy();
    }

};

#endif // SYSTEM_H