// Vector.h
#ifndef VECTOR_H
#define VECTOR_H

#include <ostream>

class Vector {
private:
    double m_x, m_y, m_z;

public:
    // Constructors
    Vector();
    Vector(double x, double y, double z);

    // Getter and Setter methods
    double getX() const;
    void setX(double x);
    double getY() const;
    void setY(double y);
    double getZ() const;
    void setZ(double z);

    // Vector operations
    Vector operator+(const Vector& v) const;
    Vector operator-(const Vector& v) const;
    Vector operator*(double scalar) const;
    double dot(const Vector& v) const;
    Vector cross(const Vector& v) const;

    // Overload the << operator
    friend std::ostream& operator<<(std::ostream& os, const Vector& v);
};

#endif // VECTOR_H
