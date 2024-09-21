// Animate.h
#ifndef ANIMATE_H
#define ANIMATE_H

#include <GLFW/glfw3.h>
#include <GL/glu.h>
#include <tuple>

#include "Vector3.h"

std::tuple<GLFWwindow*, GLUquadric*> initAnimation();
void endAnimation(GLFWwindow* window, GLUquadric* quadric);
void Animate(System system, GLFWwindow* window, GLUquadric* quadric);

#endif // ANIMATE_H