#include <iostream>
#include <fstream>
#include "Vector.h"
#include "Point.h"

int main() {
    
    // Initial position, velocity, and acceleration
    Vector initialPosition(0, 0, 0);
    Vector initialVelocity(0, 10, 0);
    Vector initialAcceleration(0, 0, 0); // gravity-like acceleration
    Vector force(5, -10, 0); // A force of 10 units in the x direction
    double mass = 2.0; // Mass of 2 units

    // Create a Point object
    Point p(initialPosition, initialVelocity, initialAcceleration, mass);
    p.setForce(force);

    // Time step
    double deltaTime = 0.05;

    // Simulation time
    double totalTime = 5;

    // Open a file to write the data
    std::ofstream file("./out/simulation_results.csv");
    file << "Time,X_Position,Y_Position,Z_Position\n";

    // Simulate
    for (int i = 0; i <= totalTime/deltaTime; ++i) {
        double t = i * deltaTime;
        Vector pos = p.getPosition();
        file << t << "," << pos.getX() << "," << pos.getY() << "," << pos.getZ() << "\n";
        p.update(deltaTime);
    }

    file.close();
    return 0;
}