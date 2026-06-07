#include "PhysicsWorld.h"
#include "CircleBody.h"
#include "CollisionDetector.h"
#include "CollisionResolver.h"
#include "Constants.h"
#include <cmath>

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
            Physics::Manifold manifold;
            if (Physics::CollisionDetector::detect(bodies[i].get(), bodies[j].get(), manifold))
            {
                Physics::CollisionResolver::resolve(manifold);
            }
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

            // Floor friction: reduce horizontal velocity
            vel.x *= (1.f - Constants::FRICTION_DYNAMIC);

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