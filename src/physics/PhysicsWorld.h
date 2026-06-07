#pragma once
#include <vector>
#include <memory>
#include "RigidBody.h"
#include "Constraint.h"
#include "SpatialGrid.h"

class PhysicsWorld
{
public:
    PhysicsWorld(float width = 800.f, float height = 600.f);

    void update(float dt);

    void addBody(std::unique_ptr<Physics::RigidBody> body);
    void addConstraint(std::unique_ptr<Physics::Constraint> constraint);

    const std::vector<std::unique_ptr<Physics::RigidBody>>& getBodies() const;
    const std::vector<std::unique_ptr<Physics::Constraint>>& getConstraints() const;

private:
    std::vector<std::unique_ptr<Physics::RigidBody>> bodies;
    std::vector<std::unique_ptr<Physics::Constraint>> constraints;
    Physics::SpatialGrid grid;

    // Collision helpers
    void applyGravity();
    void integrateAll(float dt);
    void detectAndResolveCollisions();
};