#pragma once

#include "Shape.h"
#include <SFML/System/Vector2.hpp>
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
        std::array<sf::Vector2f, 4> getVertices(const sf::Vector2f& position, float angle) const;

        // Returns the 2 unique edge normals (perpendicular to each edge direction)
        std::array<sf::Vector2f, 2> getEdgeNormals(float angle) const;

    private:
        float width;
        float height;
        float boundingRadius; // half-diagonal, cached
    };
}
