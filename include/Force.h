// Force.h
#ifndef FORCE_H
#define FORCE_H

class System; // Forward declaration

class Force {
public:
    virtual ~Force() = default;
    virtual void applyForce(System& system) const = 0;
};

#endif // FORCE_H