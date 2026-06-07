#pragma once

namespace Physics
{
    enum class ShapeType
    {
        Circle,
        Box // reserved for Phase 3
    };

    class Shape
    {
    public:
        virtual ~Shape() = default;

        virtual ShapeType getType() const = 0;

        // Bounding radius for broad-phase collision checks
        virtual float getBoundingRadius() const = 0;
    };
}
