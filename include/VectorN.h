// VectorN.h
#ifndef VECTORN_H
#define VECTORN_H

#include <ostream>
#include <array>

template<std::size_t N>
class VectorN {
private:
    std::array<double, N> data;  // Fixed-size array of doubles

public:
    // Default constructor - initializes all elements to zero
    VectorN() { data.fill(0.0); }

    // Constructor to initialize the vector with specific values
    VectorN(std::initializer_list<double> init) {
        if (init.size() > N) {
            throw std::invalid_argument("Initializer list size exceeds vector size");
        }
        std::copy(init.begin(), init.end(), data.begin());
    }

    // Access element by index (with bounds checking)
    double& operator[](std::size_t index) {
        if (index >= N) {
            throw std::out_of_range("Index out of bounds");
        }
        return data[index];
    }
    const double& operator[](std::size_t index) const {
        if (index >= N) {
            throw std::out_of_range("Index out of bounds");
        }
        return data[index];
    }

    // Get the size of the vector
    std::size_t size() const {
        return N;
    }

    // Overload + operator for vector addition
    VectorN<N> operator+(const VectorN<N>& other) const {
        VectorN<N> result;
        for (std::size_t i = 0; i < N; ++i) {
            result[i] = data[i] + other[i];
        }
        return result;
    }

    // Overload - operator for vector subtraction
    VectorN<N> operator-(const VectorN<N>& other) const {
        VectorN<N> result;
        for (std::size_t i = 0; i < N; ++i) {
            result[i] = data[i] - other[i];
        }
        return result;
    }

    // Overload * operator for scalar multiplication
    VectorN<N> operator*(double scalar) const {
        VectorN<N> result;
        for (std::size_t i = 0; i < N; ++i) {
            result[i] = data[i] * scalar;
        }
        return result;
    }
    friend VectorN<N> operator*(double scalar, const VectorN<N>& v) {
        VectorN<N> result;
        for (std::size_t i = 0; i < N; ++i) {
            result[i] = scalar * v[i];  // Scalar pre-multiplication
        }
        return result;
    }

    // Output the vector components
    friend std::ostream& operator<<(std::ostream& os, const VectorN<N>& vec) {
        for (std::size_t i = 0; i < N; ++i) {
            os << vec[i];
            if (i < N - 1) os << "\n";
        }
        return os;
    }

};

#endif // VECTORN_H