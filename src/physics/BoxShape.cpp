#include "BoxShape.h"
#include <cmath>

namespace Physics
{
    BoxCollider::BoxCollider(float width, float height)
        : width(width), height(height)
    {
        // Half-diagonal = sqrt((w/2)^2 + (h/2)^2)
        float hw = width / 2.f;
        float hh = height / 2.f;
        boundingRadius = std::sqrt(hw * hw + hh * hh);
    }

    ShapeType BoxCollider::getType() const
    {
        return ShapeType::Box;
    }

    float BoxCollider::getBoundingRadius() const
    {
        return boundingRadius;
    }

    float BoxCollider::computeMomentOfInertia(float mass) const
    {
        // Solid rectangle: I = 1/12 * m * (w^2 + h^2)
        return (1.f / 12.f) * mass * (width * width + height * height);
    }

    float BoxCollider::getWidth() const
    {
        return width;
    }

    float BoxCollider::getHeight() const
    {
        return height;
    }

    std::array<Vector2, 4> BoxCollider::getVertices(const Vector2& position, float angle) const
    {
        float hw = width / 2.f;
        float hh = height / 2.f;

        float cosA = std::cos(angle);
        float sinA = std::sin(angle);

        // Local-space corners, rotated then translated
        std::array<Vector2, 4> vertices;
        Vector2 corners[4] = {
            {-hw, -hh},
            { hw, -hh},
            { hw,  hh},
            {-hw,  hh}
        };

        for (int i = 0; i < 4; i++)
        {
            vertices[i] = {
                position.x + corners[i].x * cosA - corners[i].y * sinA,
                position.y + corners[i].x * sinA + corners[i].y * cosA
            };
        }

        return vertices;
    }

    std::array<Vector2, 2> BoxCollider::getEdgeNormals(float angle) const
    {
        float cosA = std::cos(angle);
        float sinA = std::sin(angle);

        // The two unique normals for a rectangle are the rotated X and Y axes
        return {
            Vector2{ cosA, sinA},   // normal along rotated X-axis
            Vector2{-sinA, cosA}    // normal along rotated Y-axis
        };
    }
}

