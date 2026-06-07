#include "CircleBody.h"

namespace Physics
{
    CircleCollider::CircleCollider(float radius)
        : radius(radius)
    {
    }

    ShapeType CircleCollider::getType() const
    {
        return ShapeType::Circle;
    }

    float CircleCollider::getBoundingRadius() const
    {
        return radius;
    }

    float CircleCollider::computeMomentOfInertia(float mass) const
    {
        // Solid cylinder/circle: I = 1/2 * m * r^2
        return 0.5f * mass * radius * radius;
    }

    float CircleCollider::getRadius() const
    {
        return radius;
    }
}
