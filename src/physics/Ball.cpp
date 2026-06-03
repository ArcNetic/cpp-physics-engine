#include "Ball.h"
#include <cmath>

Ball::Ball(float x, float y, float radius) : radius(radius)
{
    shape.setRadius(radius);
    shape.setPosition({x, y});

    velocity = {0.f, 0.f};
}

void Ball::update(float dt, float gravity, float floorY, float restitution, float sleepThreshold)
{
    velocity.y += gravity * dt;

    sf::Vector2f position = shape.getPosition();

    position += velocity * dt;

    // Floor collision
    if (position.y + radius >= floorY)
    {
        position.y = floorY - radius;

        velocity.y *= -restitution;

        if (std::abs(velocity.y) < sleepThreshold)
        {
            velocity.y = 0.f;
        }
    }

    shape.setPosition(position);
}

void Ball::render(sf::RenderWindow& window)
{
    window.draw(shape);
}

void Ball::setColor(const sf::Color& color)
{
    shape.setFillColor(color);
}

void Ball::setPosition(const sf::Vector2f& position)
{
    shape.setPosition(position);
}

sf::Vector2f Ball::getPosition() const
{
    return shape.getPosition();
}

float Ball::getRadius() const
{
    return radius;
}

sf::Vector2f Ball::getVelocity() const
{
    return velocity;
}
void Ball::setVelocity(const sf::Vector2f& newvelocity)
{
    velocity = newvelocity;
}