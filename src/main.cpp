#include <iostream>
#include <fstream>
#include "Vector.h"
#include "Point.h"

int main() {
    
    // Initial position, velocity, and acceleration
    Vector position(0, 0, 0);
    Vector velocity(15, 15, 0);
    Vector acceleration(0, -9.8, 0); // gravity-like acceleration

    // Create a Point object
    Point p(position, velocity, acceleration);

    // Time step
    double deltaTime = 0.05;

    // Simulationtim
    double totalTime = 5;

    // Open a file to write the data
    std::ofstream file("../out/simulation_results.csv");
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