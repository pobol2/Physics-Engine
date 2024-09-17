#include <GLFW/glfw3.h>
#include <GL/glu.h>
#include <iostream>
#include <cmath> // For sin function
#include <tuple>

#include "Vector.h"

std::tuple<GLFWwindow*, GLUquadric*> initAnimation() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
    }

    GLFWwindow* window = glfwCreateWindow(1200, 800, "Sphere Example", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }

    glfwMakeContextCurrent(window);
    glEnable(GL_DEPTH_TEST);

    // Set up projection and view matrices
    glMatrixMode(GL_PROJECTION);
    gluPerspective(45.0, 1200.0 / 800.0, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    GLUquadric* quadric = gluNewQuadric();

    return std::make_tuple(window, quadric);
}

void endAnimation(GLFWwindow* window, GLUquadric* quadric) {

    gluDeleteQuadric(quadric);
    glfwDestroyWindow(window);
    glfwTerminate();

}

void Animate(GLFWwindow* window, GLUquadric* quadric, Vector pos) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();

    glTranslatef(pos.getX(), pos.getY(), pos.getZ()); // Translate sphere

    gluSphere(quadric, 0.05, 50, 50); // Draw sphere

    glPopMatrix();

    glfwSwapBuffers(window);
    glfwPollEvents();
}