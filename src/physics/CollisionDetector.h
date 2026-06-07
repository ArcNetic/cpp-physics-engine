#pragma once
#include "Vector2.h"

#include <SFML/System/Vector2.hpp>
#include "RigidBody.h"

namespace Physics
{
    // Contact information produced by narrow-phase detection
    struct Manifold
    {
        RigidBody* a = nullptr;
        RigidBody* b = nullptr;
        Vector2 normal;   // Points from A toward B
        float penetration = 0.f;
        Vector2 contactPoint; // World space contact point
    };

    class CollisionDetector
    {
    public:
        // Test two bodies for overlap; fills manifold on collision
        // Returns true if a collision was detected
        static bool detect(RigidBody* a, RigidBody* b, Manifold& out);

    private:
        static bool circleVsCircle(RigidBody* a, RigidBody* b, Manifold& out);
        static bool boxVsBox(RigidBody* a, RigidBody* b, Manifold& out);
        static bool circleVsBox(RigidBody* circle, RigidBody* box, Manifold& out);

        // SAT helpers
        struct Projection { float min; float max; };
        static Projection projectVertices(const Vector2* vertices, int count, const Vector2& axis);
        static Projection projectCircle(const Vector2& center, float radius, const Vector2& axis);
        static float dot(const Vector2& a, const Vector2& b);
        static float lengthSq(const Vector2& v);
        static Vector2 getSupportPoint(const std::array<Vector2, 4>& verts, const Vector2& dir);
    };
}

