#include <iostream>
#include <fstream>

#include <GLFW/glfw3.h>
#include <GL/glu.h>
#include <tuple>
#include <cmath> // For sin function

#include "Vector.h"
#include "Particle.h"
#include "System.h"
#include "Animate.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "DragForce.h"

int main() {

    System particleSystem;

    // Create some particles
    Particle p1(Vector(1, 0, 0), Vector(0, 3, 0), 1.0);
    Particle p2(Vector(-1, 1, 0), Vector(0, 0, 0), 1.0);

    // Add particles to the system
    particleSystem.addParticle(p1);
    particleSystem.addParticle(p2);

    // Gravity
    GravityForce gravity(9.81);
    particleSystem.addForce(gravity);

    // Spring between particles 0 and 1, rest length 5, stiffness 100
    SpringForce spring(0, 1, 0, 100);  
    particleSystem.addForce(spring);

    // Drag
    DragForce drag(10);
    particleSystem.addForce(drag);

    // Time step
    double deltaTime = 0.001;

    // Simulation time
    double totalTime = 1;

    // Open a file to write the data
    std::ofstream file("./out/simulation_results.csv");
    file << "Time,X_Position,Y_Position,Z_Position\n";

    auto [window, quadric] =  initAnimation();

    // Simulate
    for (int i = 0; i <= totalTime/deltaTime; ++i) {
        particleSystem.update(deltaTime);
        //double t = i * deltaTime;
        //Vector pos = particleSystem.getParticle(0).getPosition();
        std::cout << particleSystem.totalEnergy() << std::endl;
        //file << t << "," << pos.getX() << "," << pos.getY() << "," << pos.getZ() << "\n";

        Animate(particleSystem, window, quadric);
    }

    endAnimation(window, quadric);
    file.close();

    return 0;
}
