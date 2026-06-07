#pragma once

#include "Shape.h"

namespace Physics
{
    class CircleCollider : public Shape
    {
    public:
        explicit CircleCollider(float radius);

        ShapeType getType() const override;
        float getBoundingRadius() const override;

        float getRadius() const;

    private:
        float radius;
    };
}
