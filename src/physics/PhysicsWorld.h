#pragma once
#include <vector>
#include <memory>
#include "RigidBody.h"

class PhysicsWorld
{
public:
    void update(float dt);

    void addBody(std::unique_ptr<Physics::RigidBody> body);

    const std::vector<std::unique_ptr<Physics::RigidBody>>& getBodies() const;

private:
    std::vector<std::unique_ptr<Physics::RigidBody>> bodies;

    // Collision helpers
    void applyGravity();
    void integrateAll(float dt);
    void detectAndResolveCollisions();
};