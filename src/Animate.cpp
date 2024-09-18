#include <GLFW/glfw3.h>
#include <GL/glu.h>
#include <iostream>
#include <cmath> // For sin function
#include <tuple>

#include "System.h"
#include "Vector.h"

void setupLighting() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    
    GLfloat light_position[] = {1.0, 1.0, 1.0, 0.0}; // Directional light
    GLfloat light_ambient[] = {0.2, 0.2, 0.2, 1.0};
    GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
    
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
}

void setupMaterial() {
    GLfloat mat_ambient[] = {0.0, 0.0, 1.0, 1.0}; // Blue
    GLfloat mat_diffuse[] = {0.0, 0.0, 1.0, 1.0}; // Blue
    GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0}; // White
    GLfloat mat_shininess[] = {50.0}; // Shininess

    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}

void setupBackground() {
    glClearColor(0.1, 0.1, 0.1, 1.0); // Dark gray background
}

std::tuple<GLFWwindow*, GLUquadric*> initAnimation() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
    }

    GLFWwindow* window = glfwCreateWindow(1200, 800, "Physics Engine", nullptr, nullptr);
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

    setupBackground();

    setupLighting();
    setupMaterial();

    glShadeModel(GL_SMOOTH);  // Enable Smooth Shading
    glEnable(GL_DEPTH_TEST);  // Enable Depth Testing
    glEnable(GL_MULTISAMPLE); // Add Anti-Aliasing

    GLUquadric* quadric = gluNewQuadric();

    return std::make_tuple(window, quadric);
}

void endAnimation(GLFWwindow* window, GLUquadric* quadric) {

    gluDeleteQuadric(quadric);
    glfwDestroyWindow(window);
    glfwTerminate();

}

void Animate(System system, GLFWwindow* window, GLUquadric* quadric) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    for (size_t i = 0; i < system.size(); ++i) {

        Vector pos = system.getParticle(i).getPosition();

        glPushMatrix();
        glTranslatef(pos.getX(), pos.getY(), pos.getZ()); // Translate sphere

        gluSphere(quadric, 0.05, 50, 50); // Draw sphere

        glPopMatrix();
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
}