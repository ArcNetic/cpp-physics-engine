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

    float CircleCollider::getRadius() const
    {
        return radius;
    }
}
