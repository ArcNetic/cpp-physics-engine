#pragma once

#include <SFML/Graphics.hpp>

class Ball
{
public:
    Ball(float x, float y, float radius);

    void update(float dt, float gravity, float floorY, float restitution, float sleepThreshold);
    void render(sf::RenderWindow& window);
    sf::Vector2f getPosition() const;
    float getRadius() const;
    void setColor(const sf::Color& color);
    void setPosition(const sf::Vector2f& position);
    sf::Vector2f getVelocity() const;
    void setVelocity(const sf::Vector2f& velocity);
private:
    sf::CircleShape shape;

    sf::Vector2f velocity;

    float radius;
};