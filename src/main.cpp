#include <SFML/Graphics.hpp>
#include <optional>
#include "physics/Ball.h"
#include "physics/PhysicsWorld.h"
#include "physics/Constants.h"

int main()
{
    sf::RenderWindow window(sf::VideoMode({800, 600}), "Physics Sandbox");

    // clock for dt
    sf::Clock clock;

    // Draw floor
    sf::RectangleShape floor(sf::Vector2f({800.f, 20.f}));
    floor.setPosition({0.f, 550.f});
    floor.setFillColor(sf::Color::White);

    PhysicsWorld world;

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
                    sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                    world.addBall(Ball(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y), 50.f));
                }
            }
        }

        // Update physics
        world.update(dt);

        // Render
        window.clear();
        world.render(window);
        window.draw(floor);
        window.display();
    }

    return 0;
}