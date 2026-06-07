#pragma once

#include <SFML/Graphics.hpp>
#include "../physics/RigidBody.h"
#include "../physics/Constraint.h"
#include <vector>
#include <memory>

class Renderer
{
public:
    explicit Renderer(sf::RenderWindow& window);

    // Draw all bodies in the world
    void drawBodies(const std::vector<std::unique_ptr<Physics::RigidBody>>& bodies);

    // Draw all constraints in the world
    void drawConstraints(const std::vector<std::unique_ptr<Physics::Constraint>>& constraints);

    // Toggle debug overlay (velocity vectors, bounding circles)
    void setDebugDraw(bool enabled);
    bool isDebugDraw() const;

private:
    sf::RenderWindow& window;
    bool debugDraw = false;

    void drawCircleBody(const Physics::RigidBody& body);
    void drawBoxBody(const Physics::RigidBody& body);

    // Debug helpers
    void drawVelocityVector(const Physics::RigidBody& body);
    void drawBoundingCircle(const Physics::RigidBody& body);
};
