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


void PhysicsWorld::addBody(std::unique_ptr<Physics::RigidBody> body)
{
    bodies.push_back(std::move(body));
}

const std::vector<std::unique_ptr<Physics::RigidBody>>& PhysicsWorld::getBodies() const
{
    return bodies;
}