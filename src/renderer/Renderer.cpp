#include "Renderer.h"
#include "../physics/CircleBody.h"
#include "../physics/BoxShape.h"
#include "../physics/Constants.h"
#include <cmath>

Renderer::Renderer(sf::RenderWindow& window)
    : window(window)
{
}

void Renderer::drawBodies(const std::vector<std::unique_ptr<Physics::RigidBody>>& bodies)
{
    for (const auto& body : bodies)
    {
        const Physics::Shape* shape = body->getShape();
        if (!shape) continue;

        if (shape->getType() == Physics::ShapeType::Circle)
        {
            drawCircleBody(*body);
        }
        else if (shape->getType() == Physics::ShapeType::Box)
        {
            drawBoxBody(*body);
        }

        if (debugDraw)
        {
            drawVelocityVector(*body);
            drawBoundingCircle(*body);
        }
    }
}

void Renderer::setDebugDraw(bool enabled)
{
    debugDraw = enabled;
}

bool Renderer::isDebugDraw() const
{
    return debugDraw;
}

void Renderer::drawCircleBody(const Physics::RigidBody& body)
{
    const auto* circle = static_cast<const Physics::CircleCollider*>(body.getShape());
    float r = circle->getRadius();

    sf::CircleShape shape(r);
    shape.setOrigin({r, r});
    shape.setPosition(body.getPosition());
    shape.setFillColor(sf::Color::White);
    window.draw(shape);
}

void Renderer::drawBoxBody(const Physics::RigidBody& body)
{
    const auto* box = static_cast<const Physics::BoxCollider*>(body.getShape());

    sf::RectangleShape shape({box->getWidth(), box->getHeight()});
    shape.setOrigin({box->getWidth() / 2.f, box->getHeight() / 2.f});
    shape.setPosition(body.getPosition());
    shape.setRotation(sf::radians(body.getAngle()));
    shape.setFillColor(sf::Color::White);
    window.draw(shape);
}

void Renderer::drawVelocityVector(const Physics::RigidBody& body)
{
    sf::Vector2f pos = body.getPosition();
    sf::Vector2f vel = body.getVelocity();

    // Scale velocity for visibility
    float scale = 0.05f;
    sf::Vector2f endpoint = pos + vel * scale;

    // Draw as a thin line using a vertex array
    sf::Vertex line[] = {
        sf::Vertex{{pos.x, pos.y}, sf::Color::Green},
        sf::Vertex{{endpoint.x, endpoint.y}, sf::Color::Green}
    };
    window.draw(line, 2, sf::PrimitiveType::Lines);
}

void Renderer::drawBoundingCircle(const Physics::RigidBody& body)
{
    const Physics::Shape* shape = body.getShape();
    if (!shape) return;

    float boundingRadius = shape->getBoundingRadius();

    sf::CircleShape outline(boundingRadius);
    outline.setOrigin({boundingRadius, boundingRadius});
    outline.setPosition(body.getPosition());
    outline.setFillColor(sf::Color::Transparent);
    outline.setOutlineColor(sf::Color(255, 255, 0, 80)); // Semi-transparent yellow
    outline.setOutlineThickness(1.f);
    window.draw(outline);
}
