#pragma once

#include <SFML/Graphics.hpp>

class Ball
{
public:
    Ball(float x, float y, float radius);

    void update(float dt, float gravity, float floorY, float restitution, float sleepThreshold);
    void render(sf::RenderWindow& window);

private:
    sf::CircleShape shape;

    sf::Vector2f velocity;

    float radius;
};