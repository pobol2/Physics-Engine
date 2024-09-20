#include <iostream>

#include <GLFW/glfw3.h>
#include <GL/glu.h>
#include <tuple>
#include <cmath> // For sin function

#include "Vector.h"
#include "Particle.h"
#include "Plane.h"
#include "System.h"
#include "Animate.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "DragForce.h"

int main() {

    System System;

    // Create some particles
    Particle p1(Vector(1, 1, 1), Vector(0, 0, 0), 1.0);
    Particle p2(Vector(1, 1, -1), Vector(0, 0, 0), 1.0);
    Particle p3(Vector(-1, 1, -1), Vector(0, 0, 0), 1.0);
    Particle p4(Vector(-1, 1, 1), Vector(0, 0, 0), 1.0);
    Particle p5(Vector(1, 2, 1), Vector(0, 0, 0), 1.0);
    Particle p6(Vector(1, 2, -1), Vector(0, 0, 0), 1.0);
    Particle p7(Vector(-1, 2, -1), Vector(0, 0, 0), 1.0);
    Particle p8(Vector(-1, 2, 1), Vector(0, 0, 0), 1.0);

    // Add particles to the system
    System.addParticle(p1);
    System.addParticle(p2);
    System.addParticle(p3);
    System.addParticle(p4);
    System.addParticle(p5);
    System.addParticle(p6);
    System.addParticle(p7);
    System.addParticle(p8);

    // Create the floor plane
    double angle = 0.2;
    Plane floor(Vector(sin(angle), cos(angle), 0), Vector(0, 0, 0));
    Plane wall(Vector(-1, 0, 0), Vector(2.5*cos(angle), 0, 0));
    System.addPlane(floor);
    System.addPlane(wall);

    // Gravity
    GravityForce gravity(9.81);
    System.addForce(gravity);

    // Spring between particles 0 and 1, rest length 5, stiffness 100, damping
    double K = 150;
    SpringForce spring1(0, 1, 2, K, 5);
    SpringForce spring2(1, 2, 2, K, 5);
    SpringForce spring3(2, 3, 2, K, 5);
    SpringForce spring4(3, 0, 2, K, 5);

    SpringForce spring5(4, 5, 2, K, 5);
    SpringForce spring6(5, 6, 2, K, 5);
    SpringForce spring7(6, 7, 2, K, 5);
    SpringForce spring8(7, 4, 2, K, 5);

    SpringForce spring9(0, 4, 1, K, 5);
    SpringForce spring10(1, 5, 1, K, 5);
    SpringForce spring11(2, 6, 1, K, 5);
    SpringForce spring12(3, 7, 1, K, 5);

    SpringForce spring13(0, 7, 2.236, K, 5);
    SpringForce spring14(1, 4, 2.236, K, 5);
    SpringForce spring15(2, 5, 2.236, K, 5);
    SpringForce spring16(3, 6, 2.236, K, 5);

    SpringForce spring17(1, 3, 2.828, K, 5);
    SpringForce spring18(4, 6, 2.828, K, 5);

    System.addForce(spring1);
    System.addForce(spring2);
    System.addForce(spring3);
    System.addForce(spring4);
    System.addForce(spring5);
    System.addForce(spring6);
    System.addForce(spring7);
    System.addForce(spring8);
    System.addForce(spring9);
    System.addForce(spring10);
    System.addForce(spring11);
    System.addForce(spring12);
    System.addForce(spring13);
    System.addForce(spring14);
    System.addForce(spring15);
    System.addForce(spring16);
    System.addForce(spring17);
    System.addForce(spring18);

    // Drag
    DragForce drag(1);
    System.addForce(drag);

    // Time step
    double stepTime = 0.00001; // Physics update interval
    double frameTime = 1.0 / 60.0; // Frame time for 60 FPS

    // Simulation time
    double totalTime = 5;

    auto [window, quadric] =  initAnimation();

    // Start the timer
    double currentTime = glfwGetTime();

    // Simulate and animate
    while (!glfwWindowShouldClose(window)) {

        double startTime = currentTime;
        
        // Update the physics simulation as fast as possible
        while (currentTime - startTime < frameTime) {
            System.update(stepTime);
            currentTime = glfwGetTime();
            totalTime -= stepTime;
        }

        // Render the frame
        Animate(System, window, quadric);

        if (totalTime <= 0) {
            break;
        }
    }

    endAnimation(window, quadric);

    return 0;
}
