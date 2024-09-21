// Demo.h
#ifndef DEMO_H
#define DEMO_H

#include "Vector3.h"

class System; // Forward declaration of System

void Cube(System& system, Vector3 position, Vector3 orientation, double halfSize, double K, double C);

#endif // DEMO_H