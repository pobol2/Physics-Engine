// DragForce.h
#ifndef DRAG_FORCE_H
#define DRAG_FORCE_H

#include "Force.h"

class DragForce : public Force {
public:
    // Constructor
    DragForce(double cd) : m_cd(cd) {}

    void applyForce(System& system) const override;

private:
    double m_cd;
};

#endif // DRAG_FORCE_H
