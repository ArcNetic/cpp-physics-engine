#include "PhysicsWorld.h"
#include "CircleBody.h"
#include "Constants.h"
#include <cmath>
#include <algorithm>

void PhysicsWorld::update(float dt)
{
    applyGravity();
    integrateAll(dt);
    detectAndResolveCollisions();
    enforceFloorConstraint();
}

void PhysicsWorld::applyGravity()
{
    for (auto& body : bodies)
    {
        if (!body->isStatic())
        {
            // F = m * g (downward)
            body->applyForce({0.f, Constants::GRAVITY * body->getMass()});
        }
    }
}

void PhysicsWorld::integrateAll(float dt)
{
    for (auto& body : bodies)
    {
        body->integrate(dt);
    }
}

void PhysicsWorld::detectAndResolveCollisions()
{
    for (size_t i = 0; i < bodies.size(); i++)
    {
        for (size_t j = i + 1; j < bodies.size(); j++)
        {
            Physics::RigidBody* a = bodies[i].get();
            Physics::RigidBody* b = bodies[j].get();

            // Currently only circle-circle collisions
            const auto* shapeA = dynamic_cast<const Physics::CircleCollider*>(a->getShape());
            const auto* shapeB = dynamic_cast<const Physics::CircleCollider*>(b->getShape());
            if (!shapeA || !shapeB)
                continue;

            sf::Vector2f posA = a->getPosition();
            sf::Vector2f posB = b->getPosition();

            float radiusA = shapeA->getRadius();
            float radiusB = shapeB->getRadius();

            float dx = posB.x - posA.x;
            float dy = posB.y - posA.y;

            float distSq = dx * dx + dy * dy;
            float minDist = radiusA + radiusB;

            // Early-out: no collision
            if (distSq >= minDist * minDist)
                continue;

            float distance = std::sqrt(distSq);

            // Guard against divide-by-zero when bodies overlap exactly
            if (distance < 0.0001f)
                continue;

            float overlap = minDist - distance;
            sf::Vector2f normal{dx / distance, dy / distance};

            // --- Positional correction (mass-weighted) ---
            float totalInvMass = a->getInverseMass() + b->getInverseMass();
            if (totalInvMass > 0.f)
            {
                sf::Vector2f correction = normal * (overlap / totalInvMass);
                a->setPosition(posA - correction * a->getInverseMass());
                b->setPosition(posB + correction * b->getInverseMass());
            }

            // --- Impulse-based velocity resolution ---
            sf::Vector2f velA = a->getVelocity();
            sf::Vector2f velB = b->getVelocity();
            sf::Vector2f relativeVel = velB - velA;

            float velAlongNormal = relativeVel.x * normal.x + relativeVel.y * normal.y;

            // Already separating
            if (velAlongNormal > 0.f)
                continue;

            // Restitution: use minimum of the two bodies
            float e = std::min(a->getRestitution(), b->getRestitution());

            // Impulse scalar: j = -(1+e) * Vrel·n / (1/mA + 1/mB)
            float impulseMag = -(1.f + e) * velAlongNormal / totalInvMass;

            sf::Vector2f impulse = normal * impulseMag;
            a->setVelocity(velA - impulse * a->getInverseMass());
            b->setVelocity(velB + impulse * b->getInverseMass());
        }
    }
}

void PhysicsWorld::enforceFloorConstraint()
{
    for (auto& body : bodies)
    {
        const auto* circle = dynamic_cast<const Physics::CircleCollider*>(body->getShape());
        if (!circle)
            continue;

        sf::Vector2f pos = body->getPosition();
        float radius = circle->getRadius();

        if (pos.y + radius >= Constants::FLOOR_Y)
        {
            pos.y = Constants::FLOOR_Y - radius;
            body->setPosition(pos);

            sf::Vector2f vel = body->getVelocity();
            vel.y *= -body->getRestitution();

            // Sleep threshold: stop micro-bouncing
            if (std::abs(vel.y) < Constants::SLEEP_THRESHOLD)
            {
                vel.y = 0.f;
            }

            body->setVelocity(vel);
        }
    }
}

void PhysicsWorld::render(sf::RenderWindow& window)
{
    for (const auto& body : bodies)
    {
        const auto* circle = dynamic_cast<const Physics::CircleCollider*>(body->getShape());
        if (circle)
        {
            sf::CircleShape shape(circle->getRadius());
            shape.setOrigin({circle->getRadius(), circle->getRadius()});
            shape.setPosition(body->getPosition());
            shape.setFillColor(sf::Color::White);
            window.draw(shape);
        }
    }
}

void PhysicsWorld::addBody(std::unique_ptr<Physics::RigidBody> body)
{
    bodies.push_back(std::move(body));
}

const std::vector<std::unique_ptr<Physics::RigidBody>>& PhysicsWorld::getBodies() const
{
    return bodies;
}