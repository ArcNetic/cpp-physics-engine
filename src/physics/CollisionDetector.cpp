#include "CollisionDetector.h"
#include "CircleBody.h"
#include "BoxShape.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace Physics
{
    float CollisionDetector::dot(const Vector2& a, const Vector2& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    float CollisionDetector::lengthSq(const Vector2& v)
    {
        return v.x * v.x + v.y * v.y;
    }

    CollisionDetector::Projection CollisionDetector::projectVertices(
        const Vector2* vertices, int count, const Vector2& axis)
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
        const Vector2& center, float radius, const Vector2& axis)
    {
        float centerProj = dot(center, axis);
        return {centerProj - radius, centerProj + radius};
    }

    Vector2 CollisionDetector::getSupportPoint(const std::array<Vector2, 4>& verts, const Vector2& dir)
    {
        float maxDot = dot(verts[0], dir);
        for (int i = 1; i < 4; i++)
        {
            float d = dot(verts[i], dir);
            if (d > maxDot)
            {
                maxDot = d;
            }
        }

        Vector2 avgVert(0.f, 0.f);
        int count = 0;
        for (int i = 0; i < 4; i++)
        {
            if (std::abs(dot(verts[i], dir) - maxDot) < 0.01f)
            {
                avgVert += verts[i];
                count++;
            }
        }
        return avgVert / static_cast<float>(count);
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

        Vector2 diff = b->getPosition() - a->getPosition();
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
        out.contactPoint = a->getPosition() + out.normal * (circleA->getRadius() - out.penetration * 0.5f);
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
        Vector2 bestAxis;
        bool bestAxisFromA = true;

        // Test all 4 axes (2 from each box)
        auto testAxis = [&](const Vector2& axis, bool isA) -> bool
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
                bestAxisFromA = isA;
            }
            return true; // Overlap on this axis
        };

        for (const auto& n : normalsA)
            if (!testAxis(n, true)) return false;

        for (const auto& n : normalsB)
            if (!testAxis(n, false)) return false;

        // All axes overlap — collision detected
        // Ensure normal points from A toward B
        Vector2 aToB = b->getPosition() - a->getPosition();
        if (dot(bestAxis, aToB) < 0.f)
            bestAxis = -bestAxis;

        out.a = a;
        out.b = b;
        out.normal = bestAxis;
        out.penetration = minOverlap;

        if (bestAxisFromA)
        {
            // A is the reference shape, B is the incident shape.
            // Contact point should be the deepest point on B.
            out.contactPoint = getSupportPoint(vertsB, -bestAxis);
        }
        else
        {
            // B is the reference shape, A is the incident shape.
            // Contact point should be the deepest point on A.
            out.contactPoint = getSupportPoint(vertsA, bestAxis);
        }

        return true;
    }

    // ---- Circle vs Box (SAT) ----

    bool CollisionDetector::circleVsBox(RigidBody* circleBody, RigidBody* boxBody, Manifold& out)
    {
        const auto* circle = static_cast<const CircleCollider*>(circleBody->getShape());
        const auto* box = static_cast<const BoxCollider*>(boxBody->getShape());

        Vector2 circlePos = circleBody->getPosition();
        float radius = circle->getRadius();

        auto verts = box->getVertices(boxBody->getPosition(), boxBody->getAngle());
        auto normals = box->getEdgeNormals(boxBody->getAngle());

        float minOverlap = std::numeric_limits<float>::max();
        Vector2 bestAxis;

        auto testAxis = [&](const Vector2& axis) -> bool
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
        Vector2 closestVertex;
        for (const auto& v : verts)
        {
            float dSq = lengthSq(v - circlePos);
            if (dSq < closestDistSq)
            {
                closestDistSq = dSq;
                closestVertex = v;
            }
        }

        Vector2 vertexAxis = closestVertex - circlePos;
        float axisLen = std::sqrt(lengthSq(vertexAxis));
        if (axisLen > 0.0001f)
        {
            vertexAxis = vertexAxis / axisLen;
            if (!testAxis(vertexAxis)) return false;
        }

        // All axes overlap — collision
        Vector2 aToB = boxBody->getPosition() - circleBody->getPosition();
        if (dot(bestAxis, aToB) < 0.f)
            bestAxis = -bestAxis;

        out.a = circleBody;
        out.b = boxBody;
        out.normal = bestAxis;
        out.penetration = minOverlap;
        out.contactPoint = circlePos + bestAxis * (radius - minOverlap * 0.5f);
        return true;
    }
}

