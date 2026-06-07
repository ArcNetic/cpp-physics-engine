#include "CollisionResolver.h"
#include "Constants.h"
#include <cmath>
#include <algorithm>

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

        float totalInvMass = a->getInverseMass() + b->getInverseMass();
        if (totalInvMass <= 0.f)
            return;

        sf::Vector2f velA = a->getVelocity();
        sf::Vector2f velB = b->getVelocity();
        sf::Vector2f relVel = velB - velA;

        float velAlongNormal = relVel.x * manifold.normal.x + relVel.y * manifold.normal.y;

        // Already separating
        if (velAlongNormal > 0.f)
            return;

        float e = std::min(a->getRestitution(), b->getRestitution());

        // Impulse magnitude: j = -(1+e) * Vrel·n / (1/mA + 1/mB)
        float impulseMag = -(1.f + e) * velAlongNormal / totalInvMass;

        sf::Vector2f impulse = manifold.normal * impulseMag;
        a->setVelocity(velA - impulse * a->getInverseMass());
        b->setVelocity(velB + impulse * b->getInverseMass());
    }

    void CollisionResolver::applyFrictionImpulse(Manifold& manifold)
    {
        RigidBody* a = manifold.a;
        RigidBody* b = manifold.b;

        float totalInvMass = a->getInverseMass() + b->getInverseMass();
        if (totalInvMass <= 0.f)
            return;

        sf::Vector2f velA = a->getVelocity();
        sf::Vector2f velB = b->getVelocity();
        sf::Vector2f relVel = velB - velA;

        // Project relative velocity onto the collision normal
        float velAlongNormal = relVel.x * manifold.normal.x + relVel.y * manifold.normal.y;

        // Tangent = relative velocity minus the normal component
        sf::Vector2f tangent = relVel - manifold.normal * velAlongNormal;

        float tangentLength = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);
        if (tangentLength < 0.0001f)
            return; // No tangential velocity to reduce

        tangent = tangent / tangentLength; // Normalize

        // Tangential impulse magnitude
        float frictionMag = -(relVel.x * tangent.x + relVel.y * tangent.y) / totalInvMass;

        // Compute the normal impulse magnitude for Coulomb clamping
        float normalImpulseMag = std::abs(-(1.f + std::min(a->getRestitution(), b->getRestitution()))
                                          * velAlongNormal / totalInvMass);

        // Coulomb's law: use static friction if within threshold, otherwise dynamic
        sf::Vector2f frictionImpulse;
        if (std::abs(frictionMag) < normalImpulseMag * Constants::FRICTION_STATIC)
        {
            // Static friction: apply full tangential impulse
            frictionImpulse = tangent * frictionMag;
        }
        else
        {
            // Dynamic friction: clamp to μ_d * |normalImpulse|
            float sign = (frictionMag > 0.f) ? 1.f : -1.f;
            frictionImpulse = tangent * (sign * normalImpulseMag * Constants::FRICTION_DYNAMIC);
        }

        a->setVelocity(velA - frictionImpulse * a->getInverseMass());
        b->setVelocity(velB + frictionImpulse * b->getInverseMass());
    }
}
