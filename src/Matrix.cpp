// Matrix.cpp
#include "Matrix.h"

// Constructors
Matrix::Matrix() : m_data{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}} {}
Matrix::Matrix(double a, double b, double c, double d, double e, double f, double g, double h, double i)
    : m_data{{a, b, c}, {d, e, f}, {g, h, i}} {}


// Method to set a value at a specific position
void Matrix::set(int row, int col, double value) {
    if (row >= 0 && row < 3 && col >= 0 && col < 3) {
        m_data[row][col] = value;
    } else {
        throw std::out_of_range("Index out of bounds");
    }
}


// Method to get a value from a specific position
double Matrix::get(int row, int col) const {
    if (row >= 0 && row < 3 && col >= 0 && col < 3) {
        return m_data[row][col];
    } else {
        throw std::out_of_range("Index out of bounds");
    }
}

// Matrix-matrix multiplication
Matrix Matrix::operator*(const Matrix& M) const {
    Matrix result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) {
                sum += m_data[i][k] * M.m_data[k][j];
            }
            result.m_data[i][j] = sum;
        }
    }
    return result;
}

// Matrix-vector multiplication
Vector Matrix::operator*(const Vector& v) const {
    double x = m_data[0][0] * v.getX() + m_data[0][1] * v.getY() + m_data[0][2] * v.getZ();
    double y = m_data[1][0] * v.getX() + m_data[1][1] * v.getY() + m_data[1][2] * v.getZ();
    double z = m_data[2][0] * v.getX() + m_data[2][1] * v.getY() + m_data[2][2] * v.getZ();
    return Vector(x, y, z);
}

// Overload the << operator
std::ostream& operator<<(std::ostream& os, const Matrix& m) {
    os << m.m_data[0][0] << " " << m.m_data[0][1] << " " << m.m_data[0][2] << "\n"
       << m.m_data[1][0] << " " << m.m_data[1][1] << " " << m.m_data[1][2] << "\n"
       << m.m_data[2][0] << " " << m.m_data[2][1] << " " << m.m_data[2][2];
    return os;
}
