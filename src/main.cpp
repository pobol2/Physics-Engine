#include <iostream>

class Vector {
private:
    float m_x, m_y;

public:
    // Constructors
    Vector() : m_x(0), m_y(0) {}
    Vector(float x, float y) : m_x(x), m_y(y) {}

    // Getter and Setter for x
    float getX() const { return m_x; }
    void setX(float x) { m_x = x; }

    // Getter and Setter for y
    float getY() const { return m_y; }
    void setY(float y) { m_y = y; }

    // Addition of two vectors
    Vector operator+(const Vector& v) const {
        return Vector(m_x + v.m_x, m_y + v.m_y);
    }

    // Substraction of two vectors
    Vector operator-(const Vector& v) const {
        return Vector(m_x - v.m_x, m_y - v.m_y);
    }

    // Dot product of two vectors
    float dot(const Vector& v) const {
        return m_x * v.m_x + m_y * v.m_y;
    }

    // Overload the << operator
    friend std::ostream& operator<<(std::ostream& os, const Vector& v) {
        os << v.m_x << "\n" << v.m_y;
        return os;
    }
};

class Matrix {
private:
    float m_data[2][2];

public:
    // Constructor
    Matrix() : m_data{{0, 0}, {0, 0}} {}
    Matrix(float a, float b, float c, float d) : m_data{{a, b}, {c, d}} {}

    // Static method to create an identity matrix
    static Matrix Identity() {
        return Matrix(1, 0, 0, 1);
    }

    // Method to set a value at a specific position
    void set(int row, int col, float value) {
        if (row >= 0 && row < 2 && col >= 0 && col < 2) {
            m_data[row][col] = value;
        } else {
            throw std::out_of_range("Index out of bounds");
        }
    }

    // Method to get a value from a specific position
    float get(int row, int col) const {
        if (row >= 0 && row < 2 && col >= 0 && col < 2) {
            return m_data[row][col];
        } else {
            throw std::out_of_range("Index out of bounds");
        }
    }

    // Matrix-Matrix multiplication
    Matrix operator*(const Matrix m) const {
        float a = m_data[0][0] * m.m_data[0][0] + m_data[0][1] * m.m_data[1][0];
        float b = m_data[0][0] * m.m_data[0][1] + m_data[0][1] * m.m_data[1][1];
        float c = m_data[1][0] * m.m_data[0][0] + m_data[1][1] * m.m_data[1][0];
        float d = m_data[1][0] * m.m_data[0][1] + m_data[1][1] * m.m_data[1][1];
        return Matrix(a, b, c, d);
    }

    // Matrix-vector multiplication
    Vector operator*(const Vector v) const {
        float x = m_data[0][0] * v.getX() + m_data[0][1] * v.getY();
        float y = m_data[1][0] * v.getX() + m_data[1][1] * v.getY();
        return Vector(x, y);
    }

    // Overload the << operator
    friend std::ostream& operator<<(std::ostream& os, const Matrix& m) {
        os << m.m_data[0][0] << " " << m.m_data[0][1] << "\n" << m.m_data[1][0] << " " << m.m_data[1][1];
        return os;
    }

};

int main() {

    Matrix m(1, 0, 0, 1);
    Vector v(1, 2);
    Vector v2(0, 0);

    std::cout << m * m << std::endl;

    return 0;
}