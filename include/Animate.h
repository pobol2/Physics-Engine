#ifndef ANIMATE_H
#define ANIMATE_H

#include <GLFW/glfw3.h>
#include <GL/glu.h>
#include <tuple>

#include "Vector.h"

std::tuple<GLFWwindow*, GLUquadric*> initAnimation();
void endAnimation(GLFWwindow* window, GLUquadric* quadric);
void Animate(GLFWwindow* window, GLUquadric* quadric, Vector pos);

#endif