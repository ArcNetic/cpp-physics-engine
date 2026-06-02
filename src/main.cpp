#include <SFML/Graphics.hpp>
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
        for (Ball& ball : balls)
        {
            ball.update(dt, gravity, floorY, e, sleepThreshold);
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