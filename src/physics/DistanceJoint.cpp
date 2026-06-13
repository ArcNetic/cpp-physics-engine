#include "DistanceJoint.h"
#include <cmath>

namespace
{
    Physics::Vector2 rotate(const Physics::Vector2& v, float angleInRadians)
    {
        float c = std::cos(angleInRadians);
        float s = std::sin(angleInRadians);
        return {v.x * c - v.y * s, v.x * s + v.y * c};
    }

    float crossProduct(const Physics::Vector2& a, const Physics::Vector2& b)
    {
        return a.x * b.y - a.y * b.x;
    }
}

namespace Physics
{
    DistanceJoint::DistanceJoint(RigidBody* a, RigidBody* b, const Physics::Vector2& localAnchorA, const Physics::Vector2& localAnchorB, float distance)
        : a(a), b(b), localAnchorA(localAnchorA), localAnchorB(localAnchorB), targetDistance(distance), effectiveMass(0.f), bias(0.f)
    {
    }

    void DistanceJoint::preStep(float dt)
    {
        rA = rotate(localAnchorA, a->getAngle());
        rB = rotate(localAnchorB, b->getAngle());

        Vector2 pA = a->getPosition() + rA;
        Vector2 pB = b->getPosition() + rB;

        Vector2 dp = pB - pA;
        float dist = std::sqrt(dp.x * dp.x + dp.y * dp.y);

        if (dist > 0.0001f)
        {
            n = dp / dist;
        }
        else
        {
            n = {1.f, 0.f};
            dist = 0.f;
        }

        float rnA = crossProduct(rA, n);
        float rnB = crossProduct(rB, n);

        float invMassSum = a->getInverseMass() + b->getInverseMass() +
                           (rnA * rnA) * a->getInverseInertia() +
                           (rnB * rnB) * b->getInverseInertia();

        effectiveMass = invMassSum > 0.f ? 1.f / invMassSum : 0.f;

        // Baumgarte stabilization (soft position correction)
        float beta = 0.2f;
        float C = dist - targetDistance;
        bias = (beta / dt) * C;
    }

    void DistanceJoint::applyImpulse()
    {
        Vector2 vA = a->getVelocity() + Vector2(-a->getAngularVelocity() * rA.y, a->getAngularVelocity() * rA.x);
        Vector2 vB = b->getVelocity() + Vector2(-b->getAngularVelocity() * rB.y, b->getAngularVelocity() * rB.x);

        Vector2 relVel = vB - vA;
        float jv = relVel.x * n.x + relVel.y * n.y;

        float lambda = -effectiveMass * (jv + bias);

        Vector2 impulse = n * lambda;

        a->setVelocity(a->getVelocity() - impulse * a->getInverseMass());
        a->setAngularVelocity(a->getAngularVelocity() - crossProduct(rA, impulse) * a->getInverseInertia());

        b->setVelocity(b->getVelocity() + impulse * b->getInverseMass());
        b->setAngularVelocity(b->getAngularVelocity() + crossProduct(rB, impulse) * b->getInverseInertia());
    }

    Vector2 DistanceJoint::getWorldAnchorA() const
    {
        return a->getPosition() + rotate(localAnchorA, a->getAngle());
    }

    Vector2 DistanceJoint::getWorldAnchorB() const
    {
        return b->getPosition() + rotate(localAnchorB, b->getAngle());
    }
}

