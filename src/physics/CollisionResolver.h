#pragma once

#include "CollisionDetector.h"

namespace Physics
{
    class CollisionResolver
    {
    public:
        // Resolve a single collision manifold:
        //   1. Positional correction (push bodies apart)
        //   2. Normal impulse (bounce)
        //   3. Friction impulse (tangential slowdown)
        static void resolve(Manifold& manifold);

    private:
        // Separate overlapping bodies proportional to inverse mass
        static void correctPositions(Manifold& manifold);

        // Apply impulse along the collision normal
        static void applyNormalImpulse(Manifold& manifold);

        // Apply Coulomb friction impulse along the tangent
        static void applyFrictionImpulse(Manifold& manifold);
    };
}
