#include <iostream>

class Vector3 {
private:
    double x, y, z;

public:
    // Constructors
    Vector3() : x(0), y(0), z(0) {}
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    // Accessors
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }

    /*// Mutators
    void setX(double x) { this->x = x; }
    void setY(double y) { this->y = y; }
    void setZ(double z) { this->z = z; }*/

    // Overload the + operator for vector addition
    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    // Overload the - operator for vector subtraction
    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    // Overload the * operator for scalar multiplication
    Vector3 operator*(const double scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    // Overload the * operator for dot product
    double operator*(const Vector3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // Cross product
    Vector3 cross(const Vector3& v) const {
        return Vector3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    // Overload the << operator to print the vector as a column vector
    friend std::ostream& operator<<(std::ostream& os, const Vector3& vec) {
        os << vec.x << std::endl;
        os << vec.y << std::endl;
        os << vec.z << std::endl;
        return os;
    }
};

class Matrix3 {
private:
    double data[3][3];

public:
    // Default constructor to initialize the matrix with zeros
    Matrix3() {
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                data[i][j] = 0.0;
            }
        }
    }

    // Constructor to initialize the matrix with individual values
    Matrix3(double a11, double a12, double a13,
            double a21, double a22, double a23,
            double a31, double a32, double a33) {
        data[0][0] = a11; data[0][1] = a12; data[0][2] = a13;
        data[1][0] = a21; data[1][1] = a22; data[1][2] = a23;
        data[2][0] = a31; data[2][1] = a32; data[2][2] = a33;
    }

    // Overload the () operator for element access
    double& operator()(size_t row, size_t col) {
        if (row >= 3 || col >= 3) {
            throw std::out_of_range("Matrix indices out of range");
        }
        return data[row][col];
    }

    const double& operator()(size_t row, size_t col) const {
        if (row >= 3 || col >= 3) {
            throw std::out_of_range("Matrix indices out of range");
        }
        return data[row][col];
    }

    // Matrix-vector multiplication
    Vector3 operator*(const Vector3& vec) const {
        return Vector3(
            data[0][0] * vec.getX() + data[0][1] * vec.getY() + data[0][2] * vec.getZ(),
            data[1][0] * vec.getX() + data[1][1] * vec.getY() + data[1][2] * vec.getZ(),
            data[2][0] * vec.getX() + data[2][1] * vec.getY() + data[2][2] * vec.getZ()
        );
    }

    // Overload the << operator to print the matrix as desired
    friend std::ostream& operator<<(std::ostream& os, const Matrix3& matrix) {
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                os << matrix.data[i][j] << " ";
            }
            os << std::endl;
        }
        return os;
    }
};

int main() {
    // Initialize a 3x3 matrix
    Matrix3 matrix(1, 2, 3,
                   4, 5, 6,
                   7, 8, 9);

    // Initialize a 3D vector
    Vector3 vector(1, 2, 3);

    // Multiply matrix by vector
    Vector3 result = matrix * vector;

    // Print the results
    std::cout << "Matrix:" << std::endl;
    std::cout << matrix << std::endl;
    std::cout << "Vector:" << std::endl;
    std::cout << vector << std::endl;
    std::cout << "Result of matrix-vector multiplication:" << std::endl;
    std::cout << result << std::endl;

    return 0;
}
