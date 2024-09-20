// Demo.h
#ifndef DEMO_H
#define DEMO_H

#include "Vector.h"

class System; // Forward declaration of System

void Cube(System& system, Vector position, Vector orientation, double halfSize, double K, double C);

#endif // DEMO_H