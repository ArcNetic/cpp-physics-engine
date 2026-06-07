#include "SpatialGrid.h"
#include <algorithm>
#include <cmath>

namespace Physics
{
    SpatialGrid::SpatialGrid(float width, float height, float cellSize)
        : width(width), height(height), cellSize(cellSize)
    {
        cols = static_cast<int>(std::ceil(width / cellSize));
        rows = static_cast<int>(std::ceil(height / cellSize));

        // Ensure at least 1x1 grid
        if (cols < 1) cols = 1;
        if (rows < 1) rows = 1;

        cells.resize(cols * rows);
    }

    void SpatialGrid::update(const std::vector<std::unique_ptr<RigidBody>>& bodies)
    {
        // Clear all cells
        for (auto& cell : cells)
        {
            cell.clear();
        }

        // Insert bodies
        for (const auto& body : bodies)
        {
            addBodyToGrid(body.get());
        }
    }

    void SpatialGrid::addBodyToGrid(RigidBody* body)
    {
        if (!body->getShape()) return;

        Vector2 pos = body->getPosition();
        float radius = body->getShape()->getBoundingRadius();

        // Calculate overlapping cell range
        int minX = static_cast<int>(std::floor((pos.x - radius) / cellSize));
        int maxX = static_cast<int>(std::floor((pos.x + radius) / cellSize));
        int minY = static_cast<int>(std::floor((pos.y - radius) / cellSize));
        int maxY = static_cast<int>(std::floor((pos.y + radius) / cellSize));

        // Clamp to grid bounds
        minX = std::max(0, std::min(minX, cols - 1));
        maxX = std::max(0, std::min(maxX, cols - 1));
        minY = std::max(0, std::min(minY, rows - 1));
        maxY = std::max(0, std::min(maxY, rows - 1));

        for (int y = minY; y <= maxY; y++)
        {
            for (int x = minX; x <= maxX; x++)
            {
                int index = y * cols + x;
                cells[index].push_back(body);
            }
        }
    }

    std::vector<std::pair<RigidBody*, RigidBody*>> SpatialGrid::getPotentialCollisions() const
    {
        std::vector<std::pair<RigidBody*, RigidBody*>> pairs;

        for (const auto& cell : cells)
        {
            int count = cell.size();
            for (int i = 0; i < count; i++)
            {
                for (int j = i + 1; j < count; j++)
                {
                    RigidBody* a = cell[i];
                    RigidBody* b = cell[j];

                    // Order pointers so (a, b) and (b, a) are the same pair
                    if (a > b) std::swap(a, b);

                    pairs.push_back({a, b});
                }
            }
        }

        // Sort and remove duplicates
        std::sort(pairs.begin(), pairs.end());
        pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());

        return pairs;
    }
}

