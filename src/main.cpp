#include <iostream>

class Vector {
private:
    float m_x, m_y, m_z;

public:
    // Constructors
    Vector() : m_x(0), m_y(0), m_z(0) {}
    Vector(float x, float y, float z) : m_x(x), m_y(y), m_z(z) {}

    // Getter and Setter for x
    float getX() const { return m_x; }
    void setX(float x) { m_x = x; }

    // Getter and Setter for y
    float getY() const { return m_y; }
    void setY(float y) { m_y = y; }

    // Getter and Setter for z
    float getZ() const { return m_z; }
    void setZ(float z) { m_z = z; }

    // Addition of two vectors
    Vector operator+(const Vector& v) const {
        return Vector(m_x + v.m_x, m_y + v.m_y, m_z + v.m_z);
    }

    // Substraction of two vectors
    Vector operator-(const Vector& v) const {
        return Vector(m_x - v.m_x, m_y - v.m_y, m_z - v.m_z);
    }

    // Dot product of two vectors
    float dot(const Vector& v) const {
        return m_x * v.m_x + m_y * v.m_y + m_z * v.m_z;
    }

    // Cross product of two vectors
    Vector cross(const Vector& v) const {
        float x = m_y * v.getZ() - m_z * v.getY();
        float y = m_z * v.getX() - m_x * v.getZ();
        float z = m_x * v.getY() - m_y * v.getX();
        return Vector(x, y, z);
    }

    // Overload the << operator
    friend std::ostream& operator<<(std::ostream& os, const Vector& v) {
        os << v.m_x << "\n" << v.m_y << "\n" << v.m_z;
        return os;
    }
};

class Matrix {
protected:
    float m_data[3][3];

public:
    // Constructors
    Matrix() : m_data{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}} {}
    Matrix(float a, float b, float c, float d, float e, float f, float g, float h, float i) : m_data{{a, b, c}, {d, e, f}, {g, h, i}} {}

    // Method to set a value at a specific position
    void set(int row, int col, float value) {
        if (row >= 0 && row < 3 && col >= 0 && col < 3) {
            m_data[row][col] = value;
        } else {
            throw std::out_of_range("Index out of bounds");
        }
    }

    // Method to get a value from a specific position
    float get(int row, int col) const {
        if (row >= 0 && row < 3 && col >= 0 && col < 3) {
            return m_data[row][col];
        } else {
            throw std::out_of_range("Index out of bounds");
        }
    }

    // Matrix-matrix multiplication
    Matrix operator*(const Matrix M) const {
        Matrix result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.m_data[i][j] = 0;
                for (int k = 0; k < 3; ++k) {
                    result.m_data[i][j] += m_data[i][k] * M.m_data[k][j];
                }
            }
        }
        return result;
    }

    // Matrix-vector multiplication
    Vector operator*(const Vector& v) const {
        float x = m_data[0][0] * v.getX() + m_data[0][1] * v.getY() + m_data[0][2] * v.getZ();
        float y = m_data[1][0] * v.getX() + m_data[1][1] * v.getY() + m_data[1][2] * v.getZ();
        float z = m_data[2][0] * v.getX() + m_data[2][1] * v.getY() + m_data[2][2] * v.getZ();
        return Vector(x, y, z);
    }

    // Overload the << operator
    friend std::ostream& operator<<(std::ostream& os, const Matrix& m) {
        os << m.m_data[0][0] << " " << m.m_data[0][1] << " " << m.m_data[0][2] << "\n"
        << m.m_data[1][0] << " " << m.m_data[1][1] << " " << m.m_data[1][2] << "\n"
        << m.m_data[2][0] << " " << m.m_data[2][1] << " " << m.m_data[2][2];
        return os;
    }

};

// Define Identity as a separate class that inherits from Matrix
class Identity : public Matrix {
public:
    // Constructor for 3x3 Identity Matrix
    Identity() : Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1) {}

    // Override matrix-vector multiplication to return the same vector
    Vector operator*(const Vector& v) const {
        return v; // Identity matrix simply returns the input vector
    }

    // Override matrix-matrix multiplication to return the other matrix
    Matrix operator*(const Matrix& m) const {
        return m; // Identity matrix * matrix = matrix
    }
};


int main() {
    
    Matrix M(1, 2, 3,
            4, 5, 6,
            7, 8, 9);
    Vector v(1, 2, 3);
    Vector v2(0, 0, 0);
    Identity I;

    v2.cross(v);

    std::cout << I * v << std::endl;
    std::cout << M * I << std::endl;

    return 0;
}