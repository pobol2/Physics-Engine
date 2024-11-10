// Constraint.h
#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <memory>

class System; // Forward declaration

class Constraint {
public:
    virtual ~Constraint() = default;
    
    virtual void applyConstraint(System& system) const = 0;
};
#endif // CONSTRAINT_H
