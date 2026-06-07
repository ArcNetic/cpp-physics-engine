#include "PhysicsWorld.h"
#include "CircleBody.h"
#include "BoxShape.h"
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
        const Physics::Shape* shape = body->getShape();
        if (!shape) continue;

        sf::Vector2f pos = body->getPosition();
        float bottomExtent = 0.f;

        if (shape->getType() == Physics::ShapeType::Circle)
        {
            bottomExtent = static_cast<const Physics::CircleCollider*>(shape)->getRadius();
        }
        else if (shape->getType() == Physics::ShapeType::Box)
        {
            // For boxes, use the bounding radius as a conservative extent
            // (exact would require computing rotated vertex Y, but this is sufficient)
            const auto* box = static_cast<const Physics::BoxCollider*>(shape);
            auto verts = box->getVertices(pos, body->getAngle());
            float maxY = verts[0].y;
            for (int i = 1; i < 4; i++)
                if (verts[i].y > maxY) maxY = verts[i].y;
            bottomExtent = maxY - pos.y;
        }

        if (pos.y + bottomExtent >= Constants::FLOOR_Y)
        {
            pos.y = Constants::FLOOR_Y - bottomExtent;
            body->setPosition(pos);

            sf::Vector2f vel = body->getVelocity();
            vel.y *= -body->getRestitution();

            if (std::abs(vel.y) < Constants::SLEEP_THRESHOLD)
            {
                vel.y = 0.f;
            }

            vel.x *= (1.f - Constants::FRICTION_DYNAMIC);
            body->setVelocity(vel);
        }
    }
}

void PhysicsWorld::render(sf::RenderWindow& window)
{
    for (const auto& body : bodies)
    {
        const Physics::Shape* shape = body->getShape();
        if (!shape) continue;

        if (shape->getType() == Physics::ShapeType::Circle)
        {
            const auto* circle = static_cast<const Physics::CircleCollider*>(shape);
            sf::CircleShape drawShape(circle->getRadius());
            drawShape.setOrigin({circle->getRadius(), circle->getRadius()});
            drawShape.setPosition(body->getPosition());
            drawShape.setFillColor(sf::Color::White);
            window.draw(drawShape);
        }
        else if (shape->getType() == Physics::ShapeType::Box)
        {
            const auto* box = static_cast<const Physics::BoxCollider*>(shape);
            sf::RectangleShape drawShape({box->getWidth(), box->getHeight()});
            drawShape.setOrigin({box->getWidth() / 2.f, box->getHeight() / 2.f});
            drawShape.setPosition(body->getPosition());
            drawShape.setRotation(sf::radians(body->getAngle()));
            drawShape.setFillColor(sf::Color::White);
            window.draw(drawShape);
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