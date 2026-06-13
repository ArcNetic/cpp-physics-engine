#pragma once
#include "Vector2.h"

#include "Shape.h"
#include <array>

namespace Physics
{
    class BoxCollider : public Shape
    {
    public:
        BoxCollider(float width, float height);

        ShapeType getType() const override;
        float getBoundingRadius() const override;
        float computeMomentOfInertia(float mass) const override;

        float getWidth() const;
        float getHeight() const;

        // Returns the 4 corner vertices in world space given a position and rotation angle
        std::array<Vector2, 4> getVertices(const Vector2& position, float angle) const;

        // Returns the 2 unique edge normals (perpendicular to each edge direction)
        std::array<Vector2, 2> getEdgeNormals(float angle) const;

    private:
        float width;
        float height;
        float boundingRadius; // half-diagonal, cached
    };
}

