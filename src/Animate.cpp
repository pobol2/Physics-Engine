#include <GLFW/glfw3.h>
#include <GL/glu.h>
#include <iostream>
#include <cmath> // For sin function
#include <tuple>

#include "System.h"
#include "Vector.h"
#include "SpringForce.h"

const float ZOOM_SENSITIVITY = 0.5f;
const float ROTATION_SENSITIVITY = 0.5f;
const float TRANSLATION_SENSITIVITY = 0.01f;

float camDistance = 5.0f;
float camYaw = 0.0f;
float camPitch = 0.0f;
float camX = 0.0f, camY = 0.0f;

bool leftMouseButtonDown = false;
bool rightMouseButtonDown = false;
double lastMouseX, lastMouseY;

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    camDistance -= yoffset * ZOOM_SENSITIVITY;
    if (camDistance < 0.0f) camDistance = 0.0f; // Prevent zooming too close
    if (camDistance > 50.0f) camDistance = 50.0f; // Prevent zooming too far
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        leftMouseButtonDown = (action == GLFW_PRESS);
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        rightMouseButtonDown = (action == GLFW_PRESS);
    }
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    if (leftMouseButtonDown) {
        camYaw += (xpos - lastMouseX) * ROTATION_SENSITIVITY;
        camPitch += (ypos - lastMouseY) * ROTATION_SENSITIVITY;
    } else if (rightMouseButtonDown) {
        camX += (xpos - lastMouseX) * TRANSLATION_SENSITIVITY;
        camY -= (ypos - lastMouseY) * TRANSLATION_SENSITIVITY;
    }
    
    lastMouseX = xpos;
    lastMouseY = ypos;
}


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

void setupPlaneMaterial() {
    GLfloat mat_ambient[] = {1.0, 1.0, 1.0, 1.0}; // Light gray
    GLfloat mat_diffuse[] = {0.0, 0.0, 0.0, 0.0};
    GLfloat mat_specular[] = {0.0, 0.0, 0.0, 0.0};
    GLfloat mat_shininess[] = {0.0};

    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}


void setupParticleMaterial() {
    GLfloat mat_ambient[] = {0.0, 0.0, 1.0, 1.0}; // Blue
    GLfloat mat_diffuse[] = {0.0, 0.0, 1.0, 1.0};
    GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat mat_shininess[] = {100.0};

    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}

void setupSpringMaterial() {
    GLfloat mat_ambient[] = {1.0, 0.0, 1.0, 1.0}; // Magenta
    GLfloat mat_diffuse[] = {0.0, 0.0, 0.0, 0.0};
    GLfloat mat_specular[] = {0.0, 0.0, 0.0, 0.0};
    GLfloat mat_shininess[] = {0.0};

    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
}
void setupBackground() {
    glClearColor(0.1, 0.1, 0.1, 1.0); // Dark gray background
}

void drawSpring(const Vector& start, const Vector& end, int numCoils, float coilRadius) {
    // Number of segments to represent the spring
    int segments = numCoils * 20; // More segments for a smoother spring
    float length = (end - start).magnitude();
    float coilLength = length / numCoils;

    glLineWidth(8.0f);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= segments; ++i) {
        float t = (float)i / segments; // Parameter from 0 to 1
        float x = start.getX() + t * (end.getX() - start.getX());
        float y = start.getY() + t * (end.getY() - start.getY());
        float z = start.getZ() + t * (end.getZ() - start.getZ());

        // Calculate the sine wave for the spring
        float wave = coilRadius * sin(t * numCoils * 2.0f * M_PI);
        //glVertex3f(x, y + wave, z); // Offset y by the wave
        glVertex3f(x, y, z);
    }
    glEnd();
}

void drawPlane(const Plane& plane) {

    Vector normal = plane.getNormal();  // Plane normal
    Vector point = plane.getPoint();    // A point on the plane

    // We need two vectors orthogonal to the normal to define the plane's surface
    Vector tangent1, tangent2;

    // Create two orthogonal vectors to the normal (basis vectors in the plane)
    if (fabs(normal.getX()) > fabs(normal.getY())) {
        tangent1 = Vector(normal.getZ(), 0, -normal.getX()).normalize();  // Arbitrary orthogonal vector
    } else {
        tangent1 = Vector(0, -normal.getZ(), normal.getY()).normalize();  // Arbitrary orthogonal vector
    }
    tangent2 = normal.cross(tangent1).normalize();  // Second orthogonal vector

    glColor3f(1.0, 1.0, 1.0);
    glLineWidth(4.0f);
    glBegin(GL_LINES);
    for (GLfloat i = -2.5; i <= 2.5; i += 0.1) {
        Vector p1 = point + tangent1 * i + tangent2 * 2.5;
        Vector p2 = point + tangent1 * i - tangent2 * 2.5;
        glVertex3f(p1.getX(), p1.getY(), p1.getZ()); glVertex3f(p2.getX(), p2.getY(), p2.getZ());

        Vector p3 = point + tangent2 * i + tangent1 * 2.5;
        Vector p4 = point + tangent2 * i - tangent1 * 2.5;
        glVertex3f(p3.getX(), p3.getY(), p3.getZ()); glVertex3f(p4.getX(), p4.getY(), p4.getZ());
    }
    glEnd();
}

std::tuple<GLFWwindow*, GLUquadric*> initAnimation() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
    }

    GLFWwindow* window = glfwCreateWindow(1200, 800, "3D Camera Movement", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }

    glfwMakeContextCurrent(window);
    glEnable(GL_DEPTH_TEST);

    setupBackground();

    setupLighting();

    glShadeModel(GL_SMOOTH);  // Enable Smooth Shading
    glEnable(GL_DEPTH_TEST);  // Enable Depth Testing
    glEnable(GL_MULTISAMPLE); // Add Anti-Aliasing

    // Set the initial perspective
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1200.0 / 800.0, 0.1, 100.0);

    // Set the initial view
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, camDistance, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    glfwMakeContextCurrent(window);

    // Set callbacks
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);

    GLUquadric* quadric = gluNewQuadric();

    return std::make_tuple(window, quadric);
}

void endAnimation(GLFWwindow* window, GLUquadric* quadric) {

    gluDeleteQuadric(quadric);
    glfwDestroyWindow(window);
    glfwTerminate();

}

void Animate(System system, GLFWwindow* window, GLUquadric* quadric) {

    // Clear buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Set the model-view matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Translate the camera
    glTranslatef(camX, camY, -camDistance+5.0f);

    // Move the camera to look at the origin
    glTranslatef(0.0f, 0.0f, -5.0f);

    // Apply rotations
    glRotatef(camPitch, 1.0f, 0.0f, 0.0f);
    glRotatef(camYaw, 0.0f, 1.0f, 0.0f);

    // Draw planes
    const std::vector<Plane>& planes = system.getPlanes();
    for (const Plane& plane : planes) {
        setupPlaneMaterial();
        drawPlane(plane); // 10x10 tiles, each 1 unit in size
    }

    // Draw springs
    const auto& forces = system.getForces();
    for (const Force* force : forces) {
        if (const SpringForce* springForce = dynamic_cast<const SpringForce*>(force)) {
            size_t p1Index, p2Index;
            springForce->getParticles(p1Index, p2Index);
            Vector start = system.getParticle(p1Index).getPosition();
            Vector end = system.getParticle(p2Index).getPosition();
            setupSpringMaterial(); // Set spring material
            drawSpring(start, end, 10, 0.05); // Draw spring with 5 coils and a radius of 0.1
        }
    }

    for (size_t i = 0; i < system.size(); ++i) {

        Vector pos = system.getParticle(i).getPosition();

        glPushMatrix();
        glTranslatef(pos.getX(), pos.getY(), pos.getZ()); // Translate sphere

        setupParticleMaterial(); // Set particle material
        gluSphere(quadric, 0.05, 50, 50); // Draw sphere

        glPopMatrix();
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
}