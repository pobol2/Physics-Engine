#pragma once

// Reference equations of motion in generalized coordinates, derived by hand
// from each system's Lagrangian. They share no code with the engine, which
// integrates Cartesian coordinates with constraint forces.
#include "examples.hpp"
#include "test_support.hpp"

#include <cmath>

namespace test {
// y = (theta1, theta2, omega1, omega2).
inline Ode doublePendulumOde(const physics::examples::DoublePendulumConfig& c) {
    return [c](double, const Eigen::VectorXd& y) {
        const double t1 = y[0], t2 = y[1], w1 = y[2], w2 = y[3], m1 = c.mass1, m2 = c.mass2;
        const double l1 = c.length1, l2 = c.length2, g = c.gravity, delta = t1 - t2;
        const double denominator = 2.0 * m1 + m2 - m2 * std::cos(2.0 * delta);
        Eigen::VectorXd dy(4);
        dy << w1, w2,
            (-g * (2.0 * m1 + m2) * std::sin(t1) - m2 * g * std::sin(t1 - 2.0 * t2)
             - 2.0 * std::sin(delta) * m2 * (w2 * w2 * l2 + w1 * w1 * l1 * std::cos(delta))) / (l1 * denominator),
            (2.0 * std::sin(delta) * (w1 * w1 * l1 * (m1 + m2) + g * (m1 + m2) * std::cos(t1)
                                      + w2 * w2 * l2 * m2 * std::cos(delta))) / (l2 * denominator);
        return dy;
    };
}

// theta'' = -(g + y_pivot'') sin(theta) / l with y_pivot = a cos(w t).
inline Ode kapitzaOde(const physics::examples::KapitzaPendulumConfig& c) {
    return [c](double t, const Eigen::VectorXd& y) {
        Eigen::VectorXd dy(2);
        dy << y[1], -(c.gravity - c.pivot_amplitude * c.pivot_frequency * c.pivot_frequency
                                       * std::cos(c.pivot_frequency * t)) * std::sin(y[0]) / c.length;
        return dy;
    };
}

// theta'' = sin(theta) (Omega^2 cos(theta) - g / R), theta from the downward vertical.
inline Ode rotatingHoopOde(const physics::examples::RotatingHoopConfig& c) {
    return [c](double, const Eigen::VectorXd& y) {
        Eigen::VectorXd dy(2);
        dy << y[1], std::sin(y[0]) * (c.rotation_rate * c.rotation_rate * std::cos(y[0]) - c.gravity / c.radius);
        return dy;
    };
}

// x'' = -w0^2 (1 + h cos(gamma t)) x.
inline Ode parametricOde(const physics::examples::ParametricOscillatorConfig& c) {
    return [c](double t, const Eigen::VectorXd& y) {
        Eigen::VectorXd dy(2);
        dy << y[1], -c.natural_frequency * c.natural_frequency
                        * (1.0 + c.modulation * std::cos(c.modulation_frequency * t)) * y[0];
        return dy;
    };
}

// Energy-conserving folded chain (Calkin and March, Am. J. Phys. 57, 154, 1989):
// after the free end of a chain of length L falls a distance x from beside the
// pivot, it moves at v^2 = g x (2L - x) / (L - x). Returns the time to fall x,
// by Simpson's rule on t = integral of 2 sqrt((L - s^2) / (g (2L - s^2))) ds
// from 0 to sqrt(x), which is t = integral of dx / v with x = s^2.
inline double foldedChainFallTime(double length, double gravity, double x) {
    const auto integrand = [&](double s) {
        return 2.0 * std::sqrt((length - s * s) / (gravity * (2.0 * length - s * s)));
    };
    const int intervals = 2000;
    const double h = std::sqrt(x) / intervals;
    double sum = integrand(0.0) + integrand(std::sqrt(x));
    for (int i = 1; i < intervals; ++i) sum += (i % 2 == 1 ? 4.0 : 2.0) * integrand(i * h);
    return sum * h / 3.0;
}

inline Eigen::VectorXd vector(std::initializer_list<double> values) {
    Eigen::VectorXd result(static_cast<Eigen::Index>(values.size()));
    Eigen::Index i = 0;
    for (double value : values) result[i++] = value;
    return result;
}
} // namespace test
