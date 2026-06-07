#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include "Ball.h"

class PhysicsWorld
{

private:
    std::vector<Ball> balls;

public:
    void update(float dt);

    void render(sf::RenderWindow& window);

    void addball(const Ball &ball);

    std::vector<Ball> &getBalls();
};