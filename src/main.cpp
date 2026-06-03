#include <SFML/Graphics.hpp>
#include <iostream>
#include <optional>
#include <cmath>
#include <vector>
#include "physics/Ball.h"

int main()
{
    sf::RenderWindow window(
        sf::VideoMode({800, 600}),
        "Physics Sandbox");

    // Floor
    const float floorY = 500.f;

    // Gravity Acceleration
    const float gravity = 980.f;

    // Co-efficient of restituion
    const float e = 0.7f;

    // SleepThreshold
    const float sleepThreshold = 30.f;

    // clock for dt
    sf::Clock clock;

    // Multiple balls
    std::vector<Ball> balls;

    // Draw floor
    sf::RectangleShape floor(sf::Vector2f({800.f, 20.f}));
    floor.setPosition({0.f, 550.f});
    floor.setFillColor(sf::Color::White);

    while (window.isOpen())
    {
        float dt = clock.restart().asSeconds(); // time since the last frame

        // Events
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }

            // if mouse clicked -> spawn a ball
            if (const auto *mousepressed = event->getIf<sf::Event::MouseButtonPressed>())
            {
                if (mousepressed->button == sf::Mouse::Button::Left)
                {
                    sf::Vector2i mousePos =
                        sf::Mouse::getPosition(window);
                    balls.emplace_back(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y), 50.f);
                }
            }
        }

        // Update all balls
        for (Ball &ball : balls)
        {
            ball.update(dt, gravity, floorY, e, sleepThreshold);
        }

        // default color
        for (Ball &ball : balls)
        {
            ball.setColor(
                sf::Color::White);
        }

        // check for collisions
        for (size_t i = 0; i < balls.size(); i++)
        {
            for (size_t j = i + 1; j < balls.size(); j++)
            {
                sf::Vector2f posA = balls[i].getPosition();

                sf::Vector2f posB = balls[j].getPosition();

                float radiusA = balls[i].getRadius();

                float radiusB = balls[j].getRadius();

                float dx = posB.x - posA.x;

                float dy = posB.y - posA.y;

                float distance = std::sqrt(dx * dx + dy * dy);

                float minDistance = radiusA + radiusB;

                // Collision detected
                if (distance < minDistance)
                {
                    float overlap = minDistance - distance;
                    sf::Vector2f collisionNormal(dx / distance, dy / distance); // NOTE : divide by zero is possible (will resolve later)

                    // Push balls equally apart
                    sf::Vector2f correction = collisionNormal * (overlap / 2.f);

                    balls[i].setColor(sf::Color::Red);
                    balls[j].setColor(sf::Color::Red);

                    balls[i].setPosition(posA - correction);
                    balls[j].setPosition(posB + correction);
                }
            }
        }

        // render
        window.clear();

        window.draw(floor);
        for (Ball &ball : balls)
        {
            ball.render(window);
        }
        window.display();
    }

    return 0;
}