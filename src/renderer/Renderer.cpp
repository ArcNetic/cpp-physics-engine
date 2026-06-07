#include "Renderer.h"
#include "../physics/CircleBody.h"
#include "../physics/BoxShape.h"
#include "../physics/DistanceJoint.h"
#include "../physics/Constants.h"
#include <cmath>

namespace {
    sf::Vector2f toSFML(const Physics::Vector2& v) {
        return sf::Vector2f(v.x, v.y);
    }
}

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

void Renderer::drawConstraints(const std::vector<std::unique_ptr<Physics::Constraint>>& constraints)
{
    for (const auto& c : constraints)
    {
        if (const auto* joint = dynamic_cast<const Physics::DistanceJoint*>(c.get()))
        {
            sf::Vector2f pA = toSFML(joint->getWorldAnchorA());
            sf::Vector2f pB = toSFML(joint->getWorldAnchorB());

            sf::Vertex line[] = {
                sf::Vertex{pA, sf::Color(0, 255, 255, 150)}, // Cyan, semi-transparent
                sf::Vertex{pB, sf::Color(0, 255, 255, 150)}
            };
            window.draw(line, 2, sf::PrimitiveType::Lines);
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
    shape.setPosition(toSFML(body.getPosition()));
    shape.setFillColor(sf::Color::White);
    window.draw(shape);
}

void Renderer::drawBoxBody(const Physics::RigidBody& body)
{
    const auto* box = static_cast<const Physics::BoxCollider*>(body.getShape());

    sf::RectangleShape shape({box->getWidth(), box->getHeight()});
    shape.setOrigin({box->getWidth() / 2.f, box->getHeight() / 2.f});
    shape.setPosition(toSFML(body.getPosition()));
    shape.setRotation(sf::radians(body.getAngle()));
    shape.setFillColor(sf::Color::White);
    window.draw(shape);
}

void Renderer::drawVelocityVector(const Physics::RigidBody& body)
{
    sf::Vector2f pos = toSFML(body.getPosition());
    sf::Vector2f vel = toSFML(body.getVelocity());

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
    outline.setPosition(toSFML(body.getPosition()));
    outline.setFillColor(sf::Color::Transparent);
    outline.setOutlineColor(sf::Color(255, 255, 0, 80)); // Semi-transparent yellow
    outline.setOutlineThickness(1.f);
    window.draw(outline);
}
