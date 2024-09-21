// Matrix3.h
#ifndef MATRIX3_H
#define MATRIX3_H

#include "Vector3.h"
#include <ostream>
#include <stdexcept>

class Matrix3 {
protected:
    double m_data[3][3];

public:
    // Constructors
    Matrix3() : m_data{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}} {}
    Matrix3(double a, double b, double c, double d, double e, double f, double g, double h, double i)
        : m_data{{a, b, c}, {d, e, f}, {g, h, i}} {}


    // Method to set a value at a specific position
    void set(int row, int col, double value) {
        if (row >= 0 && row < 3 && col >= 0 && col < 3) {
            m_data[row][col] = value;
        } else {
            throw std::out_of_range("Index out of bounds");
        }
    }


    // Method to get a value from a specific position
    double get(int row, int col) const {
        if (row >= 0 && row < 3 && col >= 0 && col < 3) {
            return m_data[row][col];
        } else {
            throw std::out_of_range("Index out of bounds");
        }
    }

    // Matrix3-Matrix3 multiplication
    Matrix3 operator*(const Matrix3& M) const {
        Matrix3 result;
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

    // Matrix3-Vector3 multiplication
    Vector3 operator*(const Vector3& v) const {
        double x = m_data[0][0] * v.getX() + m_data[0][1] * v.getY() + m_data[0][2] * v.getZ();
        double y = m_data[1][0] * v.getX() + m_data[1][1] * v.getY() + m_data[1][2] * v.getZ();
        double z = m_data[2][0] * v.getX() + m_data[2][1] * v.getY() + m_data[2][2] * v.getZ();
        return Vector3(x, y, z);
    }

    // Overload the << operator
    friend std::ostream& operator<<(std::ostream& os, const Matrix3& m) {
        os << m.m_data[0][0] << " " << m.m_data[0][1] << " " << m.m_data[0][2] << "\n"
        << m.m_data[1][0] << " " << m.m_data[1][1] << " " << m.m_data[1][2] << "\n"
        << m.m_data[2][0] << " " << m.m_data[2][1] << " " << m.m_data[2][2];
        return os;
    }
};

#endif // MATRIX3_H