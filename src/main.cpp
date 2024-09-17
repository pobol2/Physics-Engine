#include <iostream>
#include <fstream>
#include "Vector.h"
#include "Particle.h"

int main() {
    
    // Initial position, velocity, and acceleration
    Vector initialPosition(0, 0, 0);
    Vector initialVelocity(0, 0, 0);
    Vector initialAcceleration(0, 0, 0);
    Vector force(0, 0, 0);
    double mass = 2.0;

    // Create a particle object
    Particle p(initialPosition, initialVelocity, initialAcceleration, mass);
    p.setForce(force);
    p.applyGravity();

    // Time step
    double deltaTime = 0.01;

    // Simulation time
    double totalTime = 1;

    // Open a file to write the data
    std::ofstream file("./out/simulation_results.csv");
    file << "Time,X_Position,Y_Position,Z_Position\n";

    // Simulate
    for (int i = 0; i <= totalTime/deltaTime; ++i) {
        double t = i * deltaTime;
        Vector pos = p.getPosition();
        std::cout << p.totalEnergy() << std::endl;
        file << t << "," << pos.getX() << "," << pos.getY() << "," << pos.getZ() << "\n";
        p.update(deltaTime);
    }

    file.close();
    return 0;
}