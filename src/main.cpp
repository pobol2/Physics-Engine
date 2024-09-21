// main.cpp
#include <iostream>

#include <GLFW/glfw3.h>
#include <cmath> // For sin function
#include <chrono> // For execution time

#include "Vector3.h"
#include "Particle.h"
#include "Plane.h"
#include "System.h"
#include "Animate.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "DragForce.h"

#include "Demo.h"

#include "VectorN.h"
#include "MatrixN.h"


int main() {

    System system;

    // Add some cubes (system, position, orientation, halfSize, K, C)
    Cube(system, Vector3(1, 1, 0), Vector3(0, 0, 0), 0.4, 50, 5);
    Cube(system, Vector3(1, 1, 1.5), Vector3(0, 0, 0), 0.4, 50, 5);
    Cube(system, Vector3(1, 1, -1.5), Vector3(0, 0, 0), 0.4, 50, 5);
    
    // Create the floor plane
    double angle = 0.1;
    system.addPlane(Vector3(sin(angle), cos(angle), 0), Vector3(0, 0, 0));
    system.addPlane(Vector3(-1, 0, 0), Vector3(2.5*cos(angle), 2.5*(1-sin(angle)), 0));

    // Gravity
    system.addGravityForce(9.81);

    // Drag
    system.addDragForce(1);

    // Time step
    double stepTime = 0.0001; // Physics update interval
    double frameTime = 1.0 / 60.0; // Frame time for 60 FPS

    // Simulation time
    double totalTime = 5;

    auto [window, quadric] =  initAnimation();

    // Start the timer
    double currentTime = glfwGetTime();

    auto start = std::chrono::high_resolution_clock::now();

    // Simulate and animate
    while (!glfwWindowShouldClose(window)) {

        double startTime = currentTime;
        
        // Update the physics simulation as fast as possible
        while (currentTime - startTime < frameTime) {
            system.update(stepTime);
            currentTime = glfwGetTime();
            totalTime -= stepTime;
        }

        // Render the frame
        Animate(system, window, quadric);

        if (totalTime <= 0) {
            break;
        }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "Time taken: " << duration.count()/1e6 << " seconds" << std::endl;

    endAnimation(window, quadric);

    return 0;
}
