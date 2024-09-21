// Identity3.h
#ifndef IDENTITY3_H
#define IDENTITY3_H

#include "Matrix3.h"

class Identity3 : public Matrix3 {
public:
    // Constructor for Identity Matrix3
    Identity3() : Matrix3(1, 0, 0, 0, 1, 0, 0, 0, 1) {}

    // Override Matrix3-Vector3 multiplication to return the same Vector3
    Vector3 operator*(const Vector3& v) const {
        return v; // Identity matrix simply returns the input Vector3
    }

    // Override Matrix3-Matrix3 multiplication to return the other Matrix3
    Matrix3 operator*(const Matrix3& m) const {
        return m; // Identity Identity3 * Matrix3 = Matrix3
    }
};

#endif // IDENTITY_H3
