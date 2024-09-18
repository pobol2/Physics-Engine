// Vector.cpp
#include "Vector.h"
#include <cmath>  // For sqrt function

// Constructors
Vector::Vector() : m_x(0), m_y(0), m_z(0) {}
Vector::Vector(double x, double y, double z) : m_x(x), m_y(y), m_z(z) {}

// Getter and Setter for x
double Vector::getX() const { return m_x; }
void Vector::setX(double x) { m_x = x; }

// Getter and Setter for y
double Vector::getY() const { return m_y; }
void Vector::setY(double y) { m_y = y; }

// Getter and Setter for z
double Vector::getZ() const { return m_z; }
void Vector::setZ(double z) { m_z = z; }

// Addition of two vectors
Vector Vector::operator+(const Vector& v) const {
    return Vector(m_x + v.m_x, m_y + v.m_y, m_z + v.m_z);
}

// Substraction of two vectors
Vector Vector::operator-(const Vector& v) const {
    return Vector(m_x - v.m_x, m_y - v.m_y, m_z - v.m_z);
}

// Scalar multiplication
Vector Vector::operator*(double scalar) const {
    return Vector(m_x * scalar, m_y * scalar, m_z * scalar);
}
Vector operator*(double scalar, const Vector& v) {
    return v * scalar; // Calls the Vector's * operator
}

// Scalar division
Vector Vector::operator/(double scalar) const {
    if (scalar == 0) {
        throw std::invalid_argument("Division by zero");
    }
    return Vector(m_x / scalar, m_y / scalar, m_z / scalar);
}

// Dot product of two vectors
double Vector::dot(const Vector& v) const {
    return m_x * v.m_x + m_y * v.m_y + m_z * v.m_z;
}

double Vector::magnitudeSquared() const {
    return this->dot(*this);
}

// Calculate the magnitude (length) of the vector
double Vector::magnitude() const {
    return std::sqrt(magnitudeSquared());
}

// Return a normalized version of the vector (unit vector)
Vector Vector::normalize() const {
    double mag = magnitude();
    if (mag == 0) {
        throw std::invalid_argument("Cannot normalize a zero vector");
    }
    return *this / mag;
}

// Cross product of two vectors
Vector Vector::cross(const Vector& v) const {
    double x = m_y * v.m_z - m_z * v.m_y;
    double y = m_z * v.m_x - m_x * v.m_z;
    double z = m_x * v.m_y - m_y * v.m_x;
    return Vector(x, y, z);
}

// Overload the << operator
std::ostream& operator<<(std::ostream& os, const Vector& v) {
    os << v.m_x << "\n" << v.m_y << "\n" << v.m_z;
    return os;
}
