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
    Particle p1(Vector(1, 0, 0), Vector(0, 0, 0), 1.0);
    Particle p2(Vector(-1, 0, 0), Vector(0, 0, 0), 1.0);
    Particle p3(Vector(0, 2, 0), Vector(0, 0, 0), 1.0);
    Particle p4(Vector(0, 1, 1), Vector(0, 0, 0), 1.0);


    // Add particles to the system
    particleSystem.addParticle(p1);
    particleSystem.addParticle(p2);
    particleSystem.addParticle(p3);
    particleSystem.addParticle(p4);

    // Gravity
    GravityForce gravity(9.81);
    //particleSystem.addForce(gravity);

    // Spring between particles 0 and 1, rest length 5, stiffness 100, damping
    SpringForce spring1(0, 1, 2, 100, 5);
    SpringForce spring2(0, 2, 1.4, 100, 5);
    SpringForce spring3(1, 2, 1.4, 100, 5);
    particleSystem.addForce(spring1);
    particleSystem.addForce(spring2);
    particleSystem.addForce(spring3);

    // Drag
    DragForce drag(1);
    //particleSystem.addForce(drag);

    // Time step
    double stepTime = 0.000001; // Physics update interval
    double frameTime = 1.0 / 60.0; // Frame time for 60 FPS

    // Simulation time
    double totalTime = 2;

    auto [window, quadric] =  initAnimation();

    // Start the timer
    double currentTime = glfwGetTime();

    // Simulate and animate
    while (!glfwWindowShouldClose(window)) {

        double startTime = currentTime;
        
        // Update the physics simulation as fast as possible
        while (currentTime - startTime < frameTime) {
            particleSystem.update(stepTime);
            currentTime = glfwGetTime();
            totalTime -= stepTime;
        }

        // Render the frame
        Animate(particleSystem, window, quadric);

        if (totalTime <= 0) {
            break;
        }
    }

    endAnimation(window, quadric);

    return 0;
}
