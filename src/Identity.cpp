// Identity.cpp
#include "Identity.h"

// Constructor for 3x3 Identity Matrix
Identity::Identity() : Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1) {}

// Override matrix-vector multiplication to return the same vector
Vector Identity::operator*(const Vector& v) const {
    return v; // Identity matrix simply returns the input vector
}

// Override matrix-matrix multiplication to return the other matrix
Matrix Identity::operator*(const Matrix& m) const {
    return m; // Identity matrix * matrix = matrix
}