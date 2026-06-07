#include <SFML/Graphics.hpp>
#include <optional>
#include <memory>
#include "physics/RigidBody.h"
#include "physics/CircleBody.h"
#include "physics/PhysicsWorld.h"
#include "physics/Constants.h"

int main()
{
    sf::RenderWindow window(sf::VideoMode({800, 600}), "Physics Sandbox");

    // clock for dt
    sf::Clock clock;

    // Draw floor
    sf::RectangleShape floor(sf::Vector2f({800.f, 20.f}));
    floor.setPosition({0.f, Constants::FLOOR_Y});
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

                    float radius = 50.f;
                    float mass = 1.f;

                    auto shape = std::make_unique<Physics::CircleCollider>(radius);
                    auto body = std::make_unique<Physics::RigidBody>(
                        sf::Vector2f(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y)),
                        mass,
                        std::move(shape)
                    );

                    world.addBody(std::move(body));
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