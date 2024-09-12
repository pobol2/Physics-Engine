// Identity.h
#ifndef IDENTITY_H
#define IDENTITY_H

#include "Matrix.h"

class Identity : public Matrix {
public:
    // Constructor for 3x3 Identity Matrix
    Identity();

    // Override matrix-vector multiplication to return the same vector
    Vector operator*(const Vector& v) const;

    // Override matrix-matrix multiplication to return the other matrix
    Matrix operator*(const Matrix& m) const;
};

#endif // IDENTITY_H
