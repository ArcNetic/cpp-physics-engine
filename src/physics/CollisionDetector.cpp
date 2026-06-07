#include "CollisionDetector.h"
#include "CircleBody.h"
#include <cmath>

namespace Physics
{
    bool CollisionDetector::detect(RigidBody* a, RigidBody* b, Manifold& out)
    {
        const Shape* shapeA = a->getShape();
        const Shape* shapeB = b->getShape();

        if (!shapeA || !shapeB)
            return false;

        // Dispatch based on shape types
        if (shapeA->getType() == ShapeType::Circle && shapeB->getType() == ShapeType::Circle)
        {
            return circleVsCircle(a, b, out);
        }

        // Unsupported shape combination
        return false;
    }

    bool CollisionDetector::circleVsCircle(RigidBody* a, RigidBody* b, Manifold& out)
    {
        const auto* circleA = static_cast<const CircleCollider*>(a->getShape());
        const auto* circleB = static_cast<const CircleCollider*>(b->getShape());

        sf::Vector2f posA = a->getPosition();
        sf::Vector2f posB = b->getPosition();

        float dx = posB.x - posA.x;
        float dy = posB.y - posA.y;

        float distSq = dx * dx + dy * dy;
        float radiusSum = circleA->getRadius() + circleB->getRadius();

        // No overlap
        if (distSq >= radiusSum * radiusSum)
            return false;

        float distance = std::sqrt(distSq);

        // Guard against exact overlap (zero distance)
        if (distance < 0.0001f)
            return false;

        out.a = a;
        out.b = b;
        out.normal = {dx / distance, dy / distance};
        out.penetration = radiusSum - distance;

        return true;
    }
}
