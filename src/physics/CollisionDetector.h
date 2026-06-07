#pragma once

#include <SFML/System/Vector2.hpp>
#include "RigidBody.h"

namespace Physics
{
    // Contact information produced by narrow-phase detection
    struct Manifold
    {
        RigidBody* a = nullptr;
        RigidBody* b = nullptr;
        sf::Vector2f normal;   // Points from A toward B
        float penetration = 0.f;
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
        static Projection projectVertices(const sf::Vector2f* vertices, int count, const sf::Vector2f& axis);
        static Projection projectCircle(const sf::Vector2f& center, float radius, const sf::Vector2f& axis);
        static float dot(const sf::Vector2f& a, const sf::Vector2f& b);
        static float lengthSq(const sf::Vector2f& v);
    };
}
