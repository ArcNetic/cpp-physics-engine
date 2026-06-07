#pragma once
#include "RigidBody.h"
#include <vector>
#include <utility>
#include <memory>

namespace Physics
{
    class SpatialGrid
    {
    public:
        SpatialGrid(float width, float height, float cellSize);

        // Clear and rebuild the grid with the current bodies
        void update(const std::vector<std::unique_ptr<RigidBody>>& bodies);

        // Get unique pairs of bodies that might be colliding (broad-phase)
        std::vector<std::pair<RigidBody*, RigidBody*>> getPotentialCollisions() const;

    private:
        float width;
        float height;
        float cellSize;
        int cols;
        int rows;

        std::vector<std::vector<RigidBody*>> cells;

        void addBodyToGrid(RigidBody* body);
    };
}
