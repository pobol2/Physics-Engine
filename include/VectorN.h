#ifndef VECTORN_H
#define VECTORN_H

#include <ostream>
#include <vector>
#include <stdexcept>
#include <initializer_list>
#include "Vector3.h"

template <typename T>
class VectorN {
private:
    std::vector<T> data;  // Dynamic size array of T

public:
    // Default constructor - initializes an empty vector
    VectorN() = default;

    // Constructor with a predefined size
    explicit VectorN(std::size_t size) {
        data.reserve(size); // Reserve space to avoid multiple allocations
    }

    // Access element by index (with bounds checking)
    T& operator[](std::size_t index) {
        if (index >= data.size()) {
            throw std::out_of_range("Index out of bounds");
        }
        return data[index];
    }
    
    const T& operator[](std::size_t index) const {
        if (index >= data.size()) {
            throw std::out_of_range("Index out of bounds");
        }
        return data[index];
    }

    // Get the size of the vector
    std::size_t size() const {
        return data.size();
    }

    // Method to append to the vector
    void append(const T& vec) {
        data.push_back(vec);
    }

    // Overload += operator for vector addition assignment
    VectorN& operator+=(const VectorN& other) {
        if (size() != other.size()) {
            throw std::invalid_argument("Vector sizes must match for addition");
        }
        for (std::size_t i = 0; i < size(); ++i) {
            data[i] += other[i]; // Use the existing operator+= from T
        }
        return *this;
    }

    // Overload -= operator for vector subtraction assignment
    VectorN& operator-=(const VectorN& other) {
        if (size() != other.size()) {
            throw std::invalid_argument("Vector sizes must match for subtraction");
        }
        for (std::size_t i = 0; i < size(); ++i) {
            data[i] -= other[i]; // Use the existing operator-= from T
        }
        return *this;
    }

    // Overload + operator for vector addition
    VectorN operator+(const VectorN& other) const {
        if (size() != other.size()) {
            throw std::invalid_argument("Vector sizes must match for addition");
        }
        VectorN result = *this; // Start with a copy of the current object
        result += other; // Use the += operator to perform the addition
        return result;
    }

    // Overload - operator for vector subtraction
    VectorN operator-(const VectorN& other) const {
        if (size() != other.size()) {
            throw std::invalid_argument("Vector sizes must match for subtraction");
        }
        VectorN result = *this; // Start with a copy of the current object
        result -= other; // Use the -= operator to perform the subtraction
        return result;
    }

    // Overload * operator for scalar multiplication
    VectorN operator*(double scalar) const {
        VectorN result(size());
        for (const T& vec : data) {
            result.data.push_back(vec * scalar); // Requires operator* defined for T
        }
        return result;
    }
    friend VectorN operator*(double scalar, const VectorN& v) {
        return v * scalar;  // Use the existing scalar multiplication
    }

    // Output the vector components
    friend std::ostream& operator<<(std::ostream& os, const VectorN& vec) {
        for (std::size_t i = 0; i < vec.size(); ++i) {
            os << vec[i];
            if (i < vec.size() - 1) os << "\n";
        }
        return os;
    }
};

// Explicit instantiation for Vector3 and double
template class VectorN<Vector3>;
template class VectorN<double>;

#endif // VECTORN_H
