#include "CollisionResolver.h"
#include "Constants.h"
#include <cmath>
#include <algorithm>

namespace
{
    float crossProduct(const sf::Vector2f& a, const sf::Vector2f& b)
    {
        return a.x * b.y - a.y * b.x;
    }

    sf::Vector2f crossProduct(float a, const sf::Vector2f& v)
    {
        return sf::Vector2f(-a * v.y, a * v.x);
    }
}

namespace Physics
{
    void CollisionResolver::resolve(Manifold& manifold)
    {
        correctPositions(manifold);
        applyNormalImpulse(manifold);
        applyFrictionImpulse(manifold);
    }

    void CollisionResolver::correctPositions(Manifold& manifold)
    {
        RigidBody* a = manifold.a;
        RigidBody* b = manifold.b;

        float totalInvMass = a->getInverseMass() + b->getInverseMass();
        if (totalInvMass <= 0.f)
            return;

        sf::Vector2f correction = manifold.normal * (manifold.penetration / totalInvMass);
        a->setPosition(a->getPosition() - correction * a->getInverseMass());
        b->setPosition(b->getPosition() + correction * b->getInverseMass());
    }

    void CollisionResolver::applyNormalImpulse(Manifold& manifold)
    {
        RigidBody* a = manifold.a;
        RigidBody* b = manifold.b;

        sf::Vector2f rA = manifold.contactPoint - a->getPosition();
        sf::Vector2f rB = manifold.contactPoint - b->getPosition();

        sf::Vector2f velA = a->getVelocity() + crossProduct(a->getAngularVelocity(), rA);
        sf::Vector2f velB = b->getVelocity() + crossProduct(b->getAngularVelocity(), rB);
        sf::Vector2f relVel = velB - velA;

        float velAlongNormal = relVel.x * manifold.normal.x + relVel.y * manifold.normal.y;

        // Already separating
        if (velAlongNormal > 0.f)
            return;

        float rnA = crossProduct(rA, manifold.normal);
        float rnB = crossProduct(rB, manifold.normal);

        float invMassSum = a->getInverseMass() + b->getInverseMass() +
                           (rnA * rnA * a->getInverseInertia()) +
                           (rnB * rnB * b->getInverseInertia());

        if (invMassSum <= 0.f)
            return;

        float e = std::min(a->getRestitution(), b->getRestitution());
        float impulseMag = -(1.f + e) * velAlongNormal / invMassSum;

        sf::Vector2f impulse = manifold.normal * impulseMag;

        a->setVelocity(a->getVelocity() - impulse * a->getInverseMass());
        a->setAngularVelocity(a->getAngularVelocity() - crossProduct(rA, impulse) * a->getInverseInertia());

        b->setVelocity(b->getVelocity() + impulse * b->getInverseMass());
        b->setAngularVelocity(b->getAngularVelocity() + crossProduct(rB, impulse) * b->getInverseInertia());
    }

    void CollisionResolver::applyFrictionImpulse(Manifold& manifold)
    {
        RigidBody* a = manifold.a;
        RigidBody* b = manifold.b;

        sf::Vector2f rA = manifold.contactPoint - a->getPosition();
        sf::Vector2f rB = manifold.contactPoint - b->getPosition();

        sf::Vector2f velA = a->getVelocity() + crossProduct(a->getAngularVelocity(), rA);
        sf::Vector2f velB = b->getVelocity() + crossProduct(b->getAngularVelocity(), rB);
        sf::Vector2f relVel = velB - velA;

        float velAlongNormal = relVel.x * manifold.normal.x + relVel.y * manifold.normal.y;

        sf::Vector2f tangent = relVel - manifold.normal * velAlongNormal;

        float tangentLength = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);
        if (tangentLength < 0.0001f)
            return;

        tangent = tangent / tangentLength;

        float rtA = crossProduct(rA, tangent);
        float rtB = crossProduct(rB, tangent);

        float invMassSum = a->getInverseMass() + b->getInverseMass() +
                           (rtA * rtA * a->getInverseInertia()) +
                           (rtB * rtB * b->getInverseInertia());

        if (invMassSum <= 0.f)
            return;

        float frictionMag = -(relVel.x * tangent.x + relVel.y * tangent.y) / invMassSum;

        // Recompute normal impulse mass sum to clamp friction
        float rnA = crossProduct(rA, manifold.normal);
        float rnB = crossProduct(rB, manifold.normal);
        float normalInvMassSum = a->getInverseMass() + b->getInverseMass() +
                                 (rnA * rnA * a->getInverseInertia()) +
                                 (rnB * rnB * b->getInverseInertia());

        float normalImpulseMag = std::abs(-(1.f + std::min(a->getRestitution(), b->getRestitution()))
                                          * velAlongNormal / normalInvMassSum);

        sf::Vector2f frictionImpulse;
        if (std::abs(frictionMag) < normalImpulseMag * Constants::FRICTION_STATIC)
        {
            frictionImpulse = tangent * frictionMag;
        }
        else
        {
            float sign = (frictionMag > 0.f) ? 1.f : -1.f;
            frictionImpulse = tangent * (sign * normalImpulseMag * Constants::FRICTION_DYNAMIC);
        }

        a->setVelocity(a->getVelocity() - frictionImpulse * a->getInverseMass());
        a->setAngularVelocity(a->getAngularVelocity() - crossProduct(rA, frictionImpulse) * a->getInverseInertia());

        b->setVelocity(b->getVelocity() + frictionImpulse * b->getInverseMass());
        b->setAngularVelocity(b->getAngularVelocity() + crossProduct(rB, frictionImpulse) * b->getInverseInertia());
    }
}
