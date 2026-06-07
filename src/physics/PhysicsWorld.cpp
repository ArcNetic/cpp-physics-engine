#include "PhysicsWorld.h"
#include "CircleBody.h"
#include "BoxShape.h"
#include "CollisionDetector.h"
#include "CollisionResolver.h"
#include "Constants.h"
#include <cmath>

PhysicsWorld::PhysicsWorld(float width, float height)
    : grid(width, height, 100.f) // 100x100 grid cells
{
}

void PhysicsWorld::update(float dt)
{
    applyGravity();

    // Pre-step constraints (calculate bias and effective masses)
    for (auto& constraint : constraints)
        constraint->preStep(dt);

    integrateAll(dt);

    // Solve collisions
    detectAndResolveCollisions();

    // Solve constraints (typically solved multiple times in a loop with collisions,
    // but a single pass after collisions works for simple joints)
    // We'll do a few iterations for stability
    for (int i = 0; i < 4; i++)
    {
        for (auto& constraint : constraints)
            constraint->applyImpulse();
    }
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
    grid.update(bodies);
    auto potentialCollisions = grid.getPotentialCollisions();

    for (const auto& pair : potentialCollisions)
    {
        Physics::Manifold manifold;
        if (Physics::CollisionDetector::detect(pair.first, pair.second, manifold))
        {
            Physics::CollisionResolver::resolve(manifold);
        }
    }
}


void PhysicsWorld::addBody(std::unique_ptr<Physics::RigidBody> body)
{
    bodies.push_back(std::move(body));
}

void PhysicsWorld::addConstraint(std::unique_ptr<Physics::Constraint> constraint)
{
    constraints.push_back(std::move(constraint));
}

const std::vector<std::unique_ptr<Physics::RigidBody>>& PhysicsWorld::getBodies() const
{
    return bodies;
}

const std::vector<std::unique_ptr<Physics::Constraint>>& PhysicsWorld::getConstraints() const
{
    return constraints;
}