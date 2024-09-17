#include <iostream>
#include <fstream>
#include "Vector.h"
#include "Particle.h"
#include "System.h"

int main() {

    System particleSystem;

    // Create some particles
    Particle p1(Vector(0, 0, 0), Vector(0, 0, 0), Vector(0, 0, 0), 1.0);
    Particle p2(Vector(1, 1, 1), Vector(0, 1, 0), Vector(0, 0, 0), 2.0);

    // Add particles to the system
    particleSystem.addParticle(p1);
    particleSystem.addParticle(p2);

    // Apply gravity
    Vector gravity(0, -9.81, 0);
    particleSystem.applyGravity(gravity);

    // Time step
    double deltaTime = 0.01;

    // Simulation time
    double totalTime = 1;

    // Open a file to write the data
    std::ofstream file("./out/simulation_results.csv");
    file << "Time,X_Position,Y_Position,Z_Position\n";

    // Simulate
    for (int i = 0; i <= totalTime/deltaTime; ++i) {
        particleSystem.update(deltaTime);
        double t = i * deltaTime;
        Vector pos = particleSystem.getParticle(0).getPosition();
        std::cout << particleSystem.totalEnergy() << std::endl;
        file << t << "," << pos.getX() << "," << pos.getY() << "," << pos.getZ() << "\n";
    }

    file.close();
    return 0;
}