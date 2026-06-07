#include "CollisionDetector.h"
#include "CircleBody.h"
#include "BoxShape.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace Physics
{
    float CollisionDetector::dot(const sf::Vector2f& a, const sf::Vector2f& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    float CollisionDetector::lengthSq(const sf::Vector2f& v)
    {
        return v.x * v.x + v.y * v.y;
    }

    CollisionDetector::Projection CollisionDetector::projectVertices(
        const sf::Vector2f* vertices, int count, const sf::Vector2f& axis)
    {
        float minProj = dot(vertices[0], axis);
        float maxProj = minProj;

        for (int i = 1; i < count; i++)
        {
            float proj = dot(vertices[i], axis);
            if (proj < minProj) minProj = proj;
            if (proj > maxProj) maxProj = proj;
        }

        return {minProj, maxProj};
    }

    CollisionDetector::Projection CollisionDetector::projectCircle(
        const sf::Vector2f& center, float radius, const sf::Vector2f& axis)
    {
        float centerProj = dot(center, axis);
        return {centerProj - radius, centerProj + radius};
    }

    // ---- Dispatch ----

    bool CollisionDetector::detect(RigidBody* a, RigidBody* b, Manifold& out)
    {
        const Shape* shapeA = a->getShape();
        const Shape* shapeB = b->getShape();

        if (!shapeA || !shapeB)
            return false;

        ShapeType typeA = shapeA->getType();
        ShapeType typeB = shapeB->getType();

        if (typeA == ShapeType::Circle && typeB == ShapeType::Circle)
            return circleVsCircle(a, b, out);

        if (typeA == ShapeType::Box && typeB == ShapeType::Box)
            return boxVsBox(a, b, out);

        // Circle vs Box (order-normalize so circle is always first arg)
        if (typeA == ShapeType::Circle && typeB == ShapeType::Box)
            return circleVsBox(a, b, out);

        if (typeA == ShapeType::Box && typeB == ShapeType::Circle)
        {
            // Swap: detect with circle first, then flip the normal
            bool hit = circleVsBox(b, a, out);
            if (hit)
            {
                std::swap(out.a, out.b);
                out.normal = -out.normal;
            }
            return hit;
        }

        return false;
    }

    // ---- Circle vs Circle ----

    bool CollisionDetector::circleVsCircle(RigidBody* a, RigidBody* b, Manifold& out)
    {
        const auto* circleA = static_cast<const CircleCollider*>(a->getShape());
        const auto* circleB = static_cast<const CircleCollider*>(b->getShape());

        sf::Vector2f diff = b->getPosition() - a->getPosition();
        float distSq = lengthSq(diff);
        float radiusSum = circleA->getRadius() + circleB->getRadius();

        if (distSq >= radiusSum * radiusSum)
            return false;

        float distance = std::sqrt(distSq);
        if (distance < 0.0001f)
            return false;

        out.a = a;
        out.b = b;
        out.normal = diff / distance;
        out.penetration = radiusSum - distance;
        return true;
    }

    // ---- Box vs Box (SAT) ----

    bool CollisionDetector::boxVsBox(RigidBody* a, RigidBody* b, Manifold& out)
    {
        const auto* boxA = static_cast<const BoxCollider*>(a->getShape());
        const auto* boxB = static_cast<const BoxCollider*>(b->getShape());

        auto vertsA = boxA->getVertices(a->getPosition(), a->getAngle());
        auto vertsB = boxB->getVertices(b->getPosition(), b->getAngle());

        auto normalsA = boxA->getEdgeNormals(a->getAngle());
        auto normalsB = boxB->getEdgeNormals(b->getAngle());

        float minOverlap = std::numeric_limits<float>::max();
        sf::Vector2f bestAxis;

        // Test all 4 axes (2 from each box)
        auto testAxis = [&](const sf::Vector2f& axis) -> bool
        {
            Projection projA = projectVertices(vertsA.data(), 4, axis);
            Projection projB = projectVertices(vertsB.data(), 4, axis);

            // Check for gap
            if (projA.max < projB.min || projB.max < projA.min)
                return false; // Separating axis found — no collision

            float overlap = std::min(projA.max - projB.min, projB.max - projA.min);
            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                bestAxis = axis;
            }
            return true; // Overlap on this axis
        };

        for (const auto& n : normalsA)
            if (!testAxis(n)) return false;

        for (const auto& n : normalsB)
            if (!testAxis(n)) return false;

        // All axes overlap — collision detected
        // Ensure normal points from A toward B
        sf::Vector2f aToB = b->getPosition() - a->getPosition();
        if (dot(bestAxis, aToB) < 0.f)
            bestAxis = -bestAxis;

        out.a = a;
        out.b = b;
        out.normal = bestAxis;
        out.penetration = minOverlap;
        return true;
    }

    // ---- Circle vs Box (SAT) ----

    bool CollisionDetector::circleVsBox(RigidBody* circleBody, RigidBody* boxBody, Manifold& out)
    {
        const auto* circle = static_cast<const CircleCollider*>(circleBody->getShape());
        const auto* box = static_cast<const BoxCollider*>(boxBody->getShape());

        sf::Vector2f circlePos = circleBody->getPosition();
        float radius = circle->getRadius();

        auto verts = box->getVertices(boxBody->getPosition(), boxBody->getAngle());
        auto normals = box->getEdgeNormals(boxBody->getAngle());

        float minOverlap = std::numeric_limits<float>::max();
        sf::Vector2f bestAxis;

        auto testAxis = [&](const sf::Vector2f& axis) -> bool
        {
            Projection projBox = projectVertices(verts.data(), 4, axis);
            Projection projCircle = projectCircle(circlePos, radius, axis);

            if (projBox.max < projCircle.min || projCircle.max < projBox.min)
                return false;

            float overlap = std::min(projBox.max - projCircle.min, projCircle.max - projBox.min);
            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                bestAxis = axis;
            }
            return true;
        };

        // Test box edge normals
        for (const auto& n : normals)
            if (!testAxis(n)) return false;

        // Test axis from circle center to closest box vertex
        float closestDistSq = std::numeric_limits<float>::max();
        sf::Vector2f closestVertex;
        for (const auto& v : verts)
        {
            float dSq = lengthSq(v - circlePos);
            if (dSq < closestDistSq)
            {
                closestDistSq = dSq;
                closestVertex = v;
            }
        }

        sf::Vector2f vertexAxis = closestVertex - circlePos;
        float axisLen = std::sqrt(lengthSq(vertexAxis));
        if (axisLen > 0.0001f)
        {
            vertexAxis = vertexAxis / axisLen;
            if (!testAxis(vertexAxis)) return false;
        }

        // All axes overlap — collision
        sf::Vector2f aToB = boxBody->getPosition() - circleBody->getPosition();
        if (dot(bestAxis, aToB) < 0.f)
            bestAxis = -bestAxis;

        out.a = circleBody;
        out.b = boxBody;
        out.normal = bestAxis;
        out.penetration = minOverlap;
        return true;
    }
}
