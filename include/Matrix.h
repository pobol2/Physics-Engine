// Matrix.h
#ifndef MATRIX_H
#define MATRIX_H

#include "Vector.h"
#include <ostream>
#include <stdexcept>

class Matrix {
protected:
    double m_data[3][3];

public:
    // Constructors
    Matrix();
    Matrix(double a, double b, double c, double d, double e, double f, double g, double h, double i);

    // Method to set a value at a specific position
    void set(int row, int col, double value);

    // Method to get a value from a specific position
    double get(int row, int col) const;

    // Matrix-matrix multiplication
    Matrix operator*(const Matrix& M) const;

    // Matrix-vector multiplication
    Vector operator*(const Vector& v) const;

    // Overload the << operator
    friend std::ostream& operator<<(std::ostream& os, const Matrix& m);
};

#endif // MATRIX_H
