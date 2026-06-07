#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
#include "RigidBody.h"

class PhysicsWorld
{
public:
    void update(float dt);

    void render(sf::RenderWindow& window);

    void addBody(std::unique_ptr<Physics::RigidBody> body);

    const std::vector<std::unique_ptr<Physics::RigidBody>>& getBodies() const;

private:
    std::vector<std::unique_ptr<Physics::RigidBody>> bodies;

    // Collision helpers
    void applyGravity();
    void integrateAll(float dt);
    void detectAndResolveCollisions();
    void enforceFloorConstraint();
};