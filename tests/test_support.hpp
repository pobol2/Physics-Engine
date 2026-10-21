#pragma once

#include <Eigen/Core>

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace test {
constexpr double pi = 3.14159265358979323846;

inline void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

inline void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        std::ostringstream error;
        error << std::setprecision(17) << message << ": got " << actual
              << ", expected " << expected << " +/- " << tolerance;
        throw std::runtime_error(error.str());
    }
}

inline void vectorNear(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected, double tolerance,
                       const std::string& message) {
    check(actual.allFinite() && expected.allFinite(), message + ": non-finite vector");
    near((actual - expected).stableNorm(), 0.0, tolerance, message);
}

template <class Callable> void rejects(Callable action, const std::string& message) {
    bool threw = false;
    try { action(); } catch (const std::exception&) { threw = true; }
    check(threw, message);
}

// Independent fixed-step RK4 for reference ODEs written in generalized
// coordinates. It shares no code with the engine's integrators.
using Ode = std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)>;
inline Eigen::VectorXd integrate(const Ode& f, Eigen::VectorXd y, double t0, double t1, double h_max) {
    const int count = static_cast<int>(std::ceil((t1 - t0) / h_max - 1e-9));
    const double h = (t1 - t0) / count;
    double t = t0;
    for (int i = 0; i < count; ++i) {
        const Eigen::VectorXd k1 = f(t, y);
        const Eigen::VectorXd k2 = f(t + h / 2, y + h / 2 * k1);
        const Eigen::VectorXd k3 = f(t + h / 2, y + h / 2 * k2);
        const Eigen::VectorXd k4 = f(t + h, y + h * k3);
        y += h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
        t = t0 + (i + 1) * h;
    }
    return y;
}

inline int run(int argc, char** argv, const std::map<std::string, std::function<void()>>& tests) {
    try {
        check(argc == 2 && tests.count(argv[1]) == 1, "Expected a known test name");
        tests.at(argv[1])();
        std::cout << argv[1] << " passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
} // namespace test
