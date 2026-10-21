#include "physics/force.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace physics {
namespace {
void require(bool valid, const char* message) {
    if (!valid) throw std::invalid_argument(message);
}

void requireIndex(std::size_t particle, std::size_t count) {
    require(particle < count, "Force particle index out of range");
}

void requireRadial(const RadialPotential& potential) {
    require(static_cast<bool>(potential.energy) && static_cast<bool>(potential.derivative),
            "Radial potential needs energy and derivative functions");
}

// Unit separation and distance; the direction is undefined at zero separation.
double separation(const Eigen::Vector3d& d, Eigen::Vector3d& direction) {
    const double r = d.stableNorm();
    require(std::isfinite(r) && r > 0.0, "Radial force is undefined at zero separation");
    direction = d / r;
    return r;
}

double finiteValue(double value, const char* message) {
    require(std::isfinite(value), message);
    return value;
}

class DrivingForce final : public Force {
public:
    DrivingForce(std::size_t particle, VectorFunction force) : particle_(particle), force_(std::move(force)) {}
    void validate(std::size_t count) const override {
        requireIndex(particle_, count);
        require(static_cast<bool>(force_), "Driving force needs a function");
    }
    void accumulate(const SystemView& system, Vectors& forces) const override {
        const Eigen::Vector3d f = force_(system.time);
        require(f.allFinite(), "Driving force must be finite");
        forces[particle_] += f;
    }

private:
    std::size_t particle_;
    VectorFunction force_;
};

class CentralForce final : public Force {
public:
    CentralForce(std::size_t particle, Motion center, RadialPotential potential)
        : particle_(particle), center_(std::move(center)), potential_(std::move(potential)) {}
    void validate(std::size_t count) const override {
        requireIndex(particle_, count);
        center_.validate();
        requireRadial(potential_);
    }
    void accumulate(const SystemView& system, Vectors& forces) const override {
        Eigen::Vector3d direction;
        const double r = separation(system.state.positions[particle_] - center_.position(system.time), direction);
        forces[particle_] -= direction * finiteValue(potential_.derivative(r), "Radial force must be finite");
    }
    bool conservative() const override { return true; }
    double potential(const SystemView& system) const override {
        const double r = (system.state.positions[particle_] - center_.position(system.time)).stableNorm();
        require(r > 0.0, "Radial potential is undefined at zero separation");
        return finiteValue(potential_.energy(r), "Radial potential must be finite");
    }
    double potentialRate(const SystemView& system) const override {
        if (center_.isStationary()) return 0.0;
        Eigen::Vector3d direction;
        const double r = separation(system.state.positions[particle_] - center_.position(system.time), direction);
        return -potential_.derivative(r) * direction.dot(center_.velocity(system.time));
    }

private:
    std::size_t particle_;
    Motion center_;
    RadialPotential potential_;
};

class PairForce final : public Force {
public:
    PairForce(std::size_t first, std::size_t second, RadialPotential potential)
        : first_(first), second_(second), potential_(std::move(potential)) {}
    void validate(std::size_t count) const override {
        requireIndex(first_, count);
        requireIndex(second_, count);
        require(first_ != second_, "A pair force needs two distinct particles");
        requireRadial(potential_);
    }
    void accumulate(const SystemView& system, Vectors& forces) const override {
        Eigen::Vector3d direction;
        const double r = separation(system.state.positions[first_] - system.state.positions[second_], direction);
        const Eigen::Vector3d f = direction * -finiteValue(potential_.derivative(r), "Pair force must be finite");
        forces[first_] += f;
        forces[second_] -= f;
    }
    bool conservative() const override { return true; }
    double potential(const SystemView& system) const override {
        const double r = (system.state.positions[first_] - system.state.positions[second_]).stableNorm();
        require(r > 0.0, "Pair potential is undefined at zero separation");
        return finiteValue(potential_.energy(r), "Pair potential must be finite");
    }

private:
    std::size_t first_, second_;
    RadialPotential potential_;
};

class Gravitation final : public Force {
public:
    Gravitation(double constant, std::vector<std::size_t> particles, double softening)
        : constant_(constant), particles_(std::move(particles)), softening_(softening) {}
    void validate(std::size_t count) const override {
        require(std::isfinite(constant_) && constant_ > 0.0, "Gravitational constant must be finite and positive");
        require(std::isfinite(softening_) && softening_ >= 0.0, "Softening must be finite and nonnegative");
        std::vector<bool> seen(count, false);
        for (std::size_t particle : particles_) {
            requireIndex(particle, count);
            require(!seen[particle], "Gravitation lists a particle twice");
            seen[particle] = true;
        }
    }
    void accumulate(const SystemView& system, Vectors& forces) const override {
        forEachPair(system, [&](std::size_t i, std::size_t j, const Eigen::Vector3d& d, double r2) {
            const double soft = r2 + softening_ * softening_;
            require(soft > 0.0, "Gravitation is undefined for coincident unsoftened particles");
            const Eigen::Vector3d f = d * (constant_ * system.masses[i] * system.masses[j] / (soft * std::sqrt(soft)));
            forces[i] -= f;
            forces[j] += f;
        });
    }
    bool conservative() const override { return true; }
    double potential(const SystemView& system) const override {
        double result = 0.0;
        forEachPair(system, [&](std::size_t i, std::size_t j, const Eigen::Vector3d&, double r2) {
            const double soft = r2 + softening_ * softening_;
            require(soft > 0.0, "Gravitation is undefined for coincident unsoftened particles");
            result -= constant_ * system.masses[i] * system.masses[j] / std::sqrt(soft);
        });
        return result;
    }

private:
    template <class Visit> void forEachPair(const SystemView& system, Visit visit) const {
        const std::size_t count = particles_.empty() ? system.masses.size() : particles_.size();
        const auto index = [&](std::size_t k) { return particles_.empty() ? k : particles_[k]; };
        for (std::size_t a = 0; a < count; ++a) {
            for (std::size_t b = a + 1; b < count; ++b) {
                const std::size_t i = index(a), j = index(b);
                const Eigen::Vector3d d = system.state.positions[i] - system.state.positions[j];
                visit(i, j, d, d.squaredNorm());
            }
        }
    }
    double constant_;
    std::vector<std::size_t> particles_;
    double softening_;
};

class QuadraticDrag final : public Force {
public:
    QuadraticDrag(std::size_t particle, double coefficient) : particle_(particle), coefficient_(coefficient) {}
    void validate(std::size_t count) const override {
        requireIndex(particle_, count);
        require(std::isfinite(coefficient_) && coefficient_ >= 0.0, "Quadratic drag must be finite and nonnegative");
    }
    void accumulate(const SystemView& system, Vectors& forces) const override {
        const Eigen::Vector3d& v = system.state.velocities[particle_];
        forces[particle_] -= v * (coefficient_ * v.stableNorm());
    }

private:
    std::size_t particle_;
    double coefficient_;
};

class LorentzForce final : public Force {
public:
    LorentzForce(std::size_t particle, double charge, FieldFunction electric, FieldFunction magnetic)
        : particle_(particle), charge_(charge), electric_(std::move(electric)), magnetic_(std::move(magnetic)) {}
    void validate(std::size_t count) const override {
        requireIndex(particle_, count);
        require(std::isfinite(charge_), "Charge must be finite");
        require(electric_ || magnetic_, "Lorentz force needs an electric or magnetic field");
    }
    void accumulate(const SystemView& system, Vectors& forces) const override {
        const Eigen::Vector3d& q = system.state.positions[particle_];
        Eigen::Vector3d field = Eigen::Vector3d::Zero();
        if (electric_) field += electric_(q, system.time);
        if (magnetic_) field += system.state.velocities[particle_].cross(magnetic_(q, system.time));
        require(field.allFinite(), "Electromagnetic field must be finite");
        forces[particle_] += field * charge_;
    }

private:
    std::size_t particle_;
    double charge_;
    FieldFunction electric_, magnetic_;
};

class FunctionForce final : public Force {
public:
    explicit FunctionForce(CustomForce definition) : definition_(std::move(definition)) {}
    void validate(std::size_t count) const override {
        require(static_cast<bool>(definition_.apply), "Custom force needs an apply function");
        require(definition_.potential || !definition_.potential_rate,
                "Custom potential rate needs a potential");
        for (std::size_t particle : definition_.particles) requireIndex(particle, count);
    }
    void accumulate(const SystemView& system, Vectors& forces) const override { definition_.apply(system, forces); }
    bool conservative() const override { return static_cast<bool>(definition_.potential); }
    double potential(const SystemView& system) const override {
        return definition_.potential ? definition_.potential(system) : 0.0;
    }
    double potentialRate(const SystemView& system) const override {
        return definition_.potential_rate ? definition_.potential_rate(system) : 0.0;
    }

private:
    CustomForce definition_;
};
} // namespace

double Force::potential(const SystemView&) const { return 0.0; }
double Force::potentialRate(const SystemView&) const { return 0.0; }

FieldFunction uniformField(const Eigen::Vector3d& value) {
    require(value.allFinite(), "Field must be finite");
    return [value](const Eigen::Vector3d&, double) { return value; };
}

RadialPotential inverseSquare(double strength) {
    require(std::isfinite(strength), "Inverse-square strength must be finite");
    return {[strength](double r) { return -strength / r; },
            [strength](double r) { return strength / (r * r); }};
}

RadialPotential lennardJones(double depth, double sigma) {
    require(std::isfinite(depth) && depth > 0.0 && std::isfinite(sigma) && sigma > 0.0,
            "Lennard-Jones parameters must be finite and positive");
    return {[depth, sigma](double r) {
                const double s6 = std::pow(sigma / r, 6);
                return 4.0 * depth * (s6 * s6 - s6);
            },
            [depth, sigma](double r) {
                const double s6 = std::pow(sigma / r, 6);
                return 4.0 * depth * (-12.0 * s6 * s6 + 6.0 * s6) / r;
            }};
}

ForcePtr drivingForce(std::size_t particle, VectorFunction force) {
    return std::make_shared<DrivingForce>(particle, std::move(force));
}

ForcePtr harmonicDrive(std::size_t particle, const Eigen::Vector3d& amplitude, double angular_frequency, double phase) {
    require(amplitude.allFinite() && std::isfinite(angular_frequency) && std::isfinite(phase),
            "Harmonic drive parameters must be finite");
    return drivingForce(particle, [=](double t) -> Eigen::Vector3d {
        return amplitude * std::cos(angular_frequency * t + phase);
    });
}

ForcePtr centralForce(std::size_t particle, Motion center, RadialPotential potential) {
    return std::make_shared<CentralForce>(particle, std::move(center), std::move(potential));
}

ForcePtr pairForce(std::size_t first, std::size_t second, RadialPotential potential) {
    return std::make_shared<PairForce>(first, second, std::move(potential));
}

ForcePtr gravitation(double constant, std::vector<std::size_t> particles, double softening) {
    return std::make_shared<Gravitation>(constant, std::move(particles), softening);
}

ForcePtr quadraticDrag(std::size_t particle, double coefficient) {
    return std::make_shared<QuadraticDrag>(particle, coefficient);
}

ForcePtr lorentzForce(std::size_t particle, double charge, FieldFunction electric, FieldFunction magnetic) {
    return std::make_shared<LorentzForce>(particle, charge, std::move(electric), std::move(magnetic));
}

ForcePtr customForce(CustomForce definition) {
    return std::make_shared<FunctionForce>(std::move(definition));
}
} // namespace physics
