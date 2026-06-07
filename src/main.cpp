#include <SFML/Graphics.hpp>
#include <optional>
#include <memory>
#include "physics/RigidBody.h"
#include "physics/CircleBody.h"
#include "physics/BoxShape.h"
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

            // Left-click -> spawn a circle
            if (const auto *mousepressed = event->getIf<sf::Event::MouseButtonPressed>())
            {
                sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                sf::Vector2f spawnPos(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));

                if (mousepressed->button == sf::Mouse::Button::Left)
                {
                    float radius = 50.f;
                    float mass = 1.f;

                    auto shape = std::make_unique<Physics::CircleCollider>(radius);
                    auto body = std::make_unique<Physics::RigidBody>(spawnPos, mass, std::move(shape));

                    world.addBody(std::move(body));
                }
                // Right-click -> spawn a box
                else if (mousepressed->button == sf::Mouse::Button::Right)
                {
                    float width = 80.f;
                    float height = 60.f;
                    float mass = 1.5f;

                    auto shape = std::make_unique<Physics::BoxCollider>(width, height);
                    auto body = std::make_unique<Physics::RigidBody>(spawnPos, mass, std::move(shape));

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