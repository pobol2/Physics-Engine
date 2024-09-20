// Demo.cpp
#include <cmath>
#include <vector>

#include "Demo.h"
#include "System.h"
#include "SpringForce.h"
#include "Vector.h"
#include "Particle.h"

void Cube(System& system, Vector position, Vector orientation, double halfSize, double K, double C) {

    // Define the initial vertices of the cube centered at the origin
    std::vector<Vector> vertices = {
        Vector( halfSize,  halfSize,  halfSize), // 0
        Vector( halfSize,  halfSize, -halfSize), // 1
        Vector( halfSize, -halfSize, -halfSize), // 2
        Vector( halfSize, -halfSize,  halfSize), // 3
        Vector(-halfSize,  halfSize,  halfSize), // 4
        Vector(-halfSize,  halfSize, -halfSize), // 5
        Vector(-halfSize, -halfSize, -halfSize), // 6
        Vector(-halfSize, -halfSize,  halfSize)  // 7
    };

    // Store the particle IDs for this cube
    std::vector<int> particleIDs;

    // Apply position and orientation and add particles
    for (auto& vertex : vertices) {
        // Apply rotation (simplified, assumes rotation around y-axis only)
        double x = vertex.getX() * cos(orientation.getY()) - vertex.getZ() * sin(orientation.getY());
        double z = vertex.getX() * sin(orientation.getY()) + vertex.getZ() * cos(orientation.getY());
        vertex.setX(x);
        vertex.setZ(z);

        // Translate to the specified position
        vertex = vertex + position;

        // Add particle to the system and store its ID
        system.addParticle(vertex, Vector(0, 0, 0), 1.0);
        particleIDs.push_back(system.getParticles().size() - 1); // Get the latest particle index
    }

    // Spring connections with adjusted lengths based on cube geometry
    double restLength = 2 * halfSize; // Length of the edges of the cube

    // Connect the vertices to form the cube
    // Edges of the cube
    system.addSpringForce(particleIDs[0], particleIDs[1], restLength, K, 5); // Edge 0-1
    system.addSpringForce(particleIDs[1], particleIDs[2], restLength, K, 5); // Edge 1-2
    system.addSpringForce(particleIDs[2], particleIDs[3], restLength, K, 5); // Edge 2-3
    system.addSpringForce(particleIDs[3], particleIDs[0], restLength, K, 5); // Edge 3-0

    system.addSpringForce(particleIDs[4], particleIDs[5], restLength, K, 5); // Edge 4-5
    system.addSpringForce(particleIDs[5], particleIDs[6], restLength, K, 5); // Edge 5-6
    system.addSpringForce(particleIDs[6], particleIDs[7], restLength, K, 5); // Edge 6-7
    system.addSpringForce(particleIDs[7], particleIDs[4], restLength, K, 5); // Edge 7-4

    // Vertical edges connecting top and bottom faces
    system.addSpringForce(particleIDs[0], particleIDs[4], restLength, K, 5); // Edge 0-4
    system.addSpringForce(particleIDs[1], particleIDs[5], restLength, K, 5); // Edge 1-5
    system.addSpringForce(particleIDs[2], particleIDs[6], restLength, K, 5); // Edge 2-6
    system.addSpringForce(particleIDs[3], particleIDs[7], restLength, K, 5); // Edge 3-7

    // Diagonal springs for each face of the cube
    double diagonalLength = sqrt(2) * restLength; // Length of the diagonal across the faces

    // Top face diagonals
    system.addSpringForce(particleIDs[0], particleIDs[2], diagonalLength, K, 5); // Diagonal 0-2
    system.addSpringForce(particleIDs[1], particleIDs[3], diagonalLength, K, 5); // Diagonal 1-3

    // Bottom face diagonals
    system.addSpringForce(particleIDs[4], particleIDs[6], diagonalLength, K, 5); // Diagonal 4-6
    system.addSpringForce(particleIDs[5], particleIDs[7], diagonalLength, K, 5); // Diagonal 5-7

    // Left face diagonals
    system.addSpringForce(particleIDs[0], particleIDs[5], diagonalLength, K, 5); // Diagonal 0-5
    system.addSpringForce(particleIDs[1], particleIDs[4], diagonalLength, K, 5); // Diagonal 1-4

    // Right face diagonals
    system.addSpringForce(particleIDs[2], particleIDs[7], diagonalLength, K, 5); // Diagonal 2-7
    system.addSpringForce(particleIDs[3], particleIDs[6], diagonalLength, K, 5); // Diagonal 3-6

    // Front face diagonals
    system.addSpringForce(particleIDs[3], particleIDs[4], diagonalLength, K, 5); // Diagonal 3-4
    system.addSpringForce(particleIDs[0], particleIDs[7], diagonalLength, K, 5); // Diagonal 0-7

    // Back face diagonals
    system.addSpringForce(particleIDs[1], particleIDs[6], diagonalLength, K, 5); // Diagonal 1-6
    system.addSpringForce(particleIDs[2], particleIDs[5], diagonalLength, K, 5); // Diagonal 2-5


}