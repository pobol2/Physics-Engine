#ifndef MATRIXN_H
#define MATRIXN_H

#include <array>
#include <stdexcept>
#include <ostream>
#include "VectorN.h"

template<std::size_t N>
class MatrixN {
private:
    std::array<std::array<double, N>, N> data;  // Fixed-size 2D array for matrix elements

public:
    // Default constructor - initializes all elements to zero
    MatrixN() {
        for (auto& row : data) {
            row.fill(0.0);
        }
    }

    // Constructor to initialize matrix with specific values (row-major order)
    MatrixN(std::initializer_list<double> init) {
        if (init.size() != N * N) {
            throw std::invalid_argument("Initializer list size must match matrix size");
        }
        auto it = init.begin();
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j < N; ++j) {
                data[i][j] = *it++;
            }
        }
    }

    // Access element by row and column (with bounds checking)
    double& operator()(std::size_t row, std::size_t col) {
        if (row >= N || col >= N) {
            throw std::out_of_range("Index out of bounds");
        }
        return data[row][col];
    }

    const double& operator()(std::size_t row, std::size_t col) const {
        if (row >= N || col >= N) {
            throw std::out_of_range("Index out of bounds");
        }
        return data[row][col];
    }

    // Get the size of the matrix (number of rows/columns)
    std::size_t size() const {
        return N;
    }

    // Overload + operator for matrix addition
    MatrixN<N> operator+(const MatrixN<N>& other) const {
        MatrixN<N> result;
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j < N; ++j) {
                result(i, j) = data[i][j] + other(i, j);
            }
        }
        return result;
    }

    // Overload - operator for matrix subtraction
    MatrixN<N> operator-(const MatrixN<N>& other) const {
        MatrixN<N> result;
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j < N; ++j) {
                result(i, j) = data[i][j] - other(i, j);
            }
        }
        return result;
    }

    // Overload * operator for scalar multiplication
    MatrixN<N> operator*(double scalar) const {
        MatrixN<N> result;
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j < N; ++j) {
                result(i, j) = data[i][j] * scalar;
            }
        }
        return result;
    }

    // Overload * operator for matrix multiplication
    MatrixN<N> operator*(const MatrixN<N>& other) const {
        MatrixN<N> result;
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j < N; ++j) {
                result(i, j) = 0.0;
                for (std::size_t k = 0; k < N; ++k) {
                    result(i, j) += data[i][k] * other(k, j);
                }
            }
        }
        return result;
    }

    // MatrixN-VectorN multiplication
    VectorN<N> operator*(const VectorN<N>& vec) const {
        if (vec.size() != N) {
            throw std::invalid_argument("Vector size must match matrix size");
        }
        VectorN<N> result;
        for (std::size_t i = 0; i < N; ++i) {
            result[i] = 0.0;
            for (std::size_t j = 0; j < N; ++j) {
                result[i] += data[i][j] * vec[j];
            }
        }
        return result;
    }

    // Output the matrix elements row by row
    friend std::ostream& operator<<(std::ostream& os, const MatrixN<N>& mat) {
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j < N; ++j) {
                os << mat(i, j) << (j == N - 1 ? "" : " ");
            }
            if (i < N - 1) os << "\n";
        }
        return os;
    }
};

#endif // MATRIXN_H
