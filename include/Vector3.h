// Vector3.h
#ifndef VECTOR3_H
#define VECTOR3_H

#include <ostream>
#include <cmath>

class Vector3 {
private:
    double m_x, m_y, m_z;

public:
    // Constructors
    Vector3() : m_x(0), m_y(0), m_z(0) {}
    Vector3(double x, double y, double z) : m_x(x), m_y(y), m_z(z) {}

    // Getter and Setter methods
    double getX() const { return m_x; }
    double getY() const { return m_y; }
    double getZ() const { return m_z; }
    void setX(double x) { m_x = x; }
    void setY(double y) { m_y = y; }
    void setZ(double z) { m_z = z; }

    // Access elements using brackets
    double operator[](std::size_t index) const {
        switch (index) {
            case 0: return m_x;
            case 1: return m_y;
            case 2: return m_z;
            default: throw std::out_of_range("Index out of bounds");
        }
    }

    // Addition assignment operator
    Vector3& operator+=(const Vector3& v) {
        m_x += v.m_x;
        m_y += v.m_y;
        m_z += v.m_z;
        return *this;
    }

    // Substraction assignment operator
    Vector3& operator-=(const Vector3& v) {
        m_x -= v.m_x;
        m_y -= v.m_y;
        m_z -= v.m_z;
        return *this;
    }

    // Addition of two Vector3
    Vector3 operator+(const Vector3& v) const {
        Vector3 result(*this);
        result += v;
        return result;
    }

    // Substraction of two Vector3
    Vector3 operator-(const Vector3& v) const {
        Vector3 result(*this);
        result -= v;
        return result;
    }

    // Scalar multiplication
    Vector3 operator*(double scalar) const {
        return Vector3(m_x * scalar, m_y * scalar, m_z * scalar);
    }
    friend Vector3 operator*(double scalar, const Vector3& v) {
        return v * scalar; // Calls the Vector3's * operator
    }

    // Scalar division
    Vector3 operator/(double scalar) const {
        if (scalar == 0) {
            throw std::invalid_argument("Division by zero");
        }
        return Vector3(m_x / scalar, m_y / scalar, m_z / scalar);
    }

    // Dot product of two Vector3
    double dot(const Vector3& v) const {
        return m_x * v.m_x + m_y * v.m_y + m_z * v.m_z;
    }

    double magnitudeSquared() const {
        return this->dot(*this);
    }

    // Calculate the magnitude (length) of the Vector3
    double magnitude() const {
        return std::sqrt(magnitudeSquared());
    }

    // Return a normalized version of the Vector3 (unit Vector3)
    Vector3 normalize() const {
        double mag = magnitude();
        if (mag == 0) {
            return *this;
            //throw std::invalid_argument("Cannot normalize a zero vector");
        }
        return *this / mag;
    }

    // Cross product of two Vector3
    Vector3 cross(const Vector3& v) const {
        return Vector3(
            m_y * v.m_z - m_z * v.m_y,
            m_z * v.m_x - m_x * v.m_z,
            m_x * v.m_y - m_y * v.m_x
        );
    }

    // Overload the << operator
    friend std::ostream& operator<<(std::ostream& os, const Vector3& v) {
        os << v.m_x << "\n" << v.m_y << "\n" << v.m_z;
        return os;
    }

};

#endif // VECTOR3_H
