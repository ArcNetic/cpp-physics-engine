#pragma once

namespace Physics
{
    class Constraint
    {
    public:
        virtual ~Constraint() = default;

        // Called before the solver iterations (e.g., to compute bias or pre-calculate Jacobians)
        virtual void preStep(float dt) = 0;

        // Called during the solver iterations to apply corrective impulses
        virtual void applyImpulse() = 0;
    };
}
