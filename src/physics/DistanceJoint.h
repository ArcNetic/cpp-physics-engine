#pragma once
#include "Vector2.h"
#include "Constraint.h"
#include "RigidBody.h"

namespace Physics
{
    class DistanceJoint : public Constraint
    {
    public:
        DistanceJoint(RigidBody* a, RigidBody* b, const Vector2& localAnchorA, const Vector2& localAnchorB, float distance);

        void preStep(float dt) override;
        void applyImpulse() override;

        RigidBody* getBodyA() const { return a; }
        RigidBody* getBodyB() const { return b; }
        Vector2 getLocalAnchorA() const { return localAnchorA; }
        Vector2 getLocalAnchorB() const { return localAnchorB; }

        Vector2 getWorldAnchorA() const;
        Vector2 getWorldAnchorB() const;

    private:
        RigidBody* a;
        RigidBody* b;

        Vector2 localAnchorA;
        Vector2 localAnchorB;

        float targetDistance;

        // Pre-calculated values per frame
        Vector2 n;      // Normal direction from A to B
        float effectiveMass; // Effective mass for the constraint
        float bias;          // Baumgarte stabilization bias

        // World space anchor points calculated in preStep
        Vector2 rA;
        Vector2 rB;
    };
}


