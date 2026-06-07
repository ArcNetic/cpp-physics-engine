#include <SFML/Graphics.hpp>
#include <optional>
#include <memory>
#include "physics/RigidBody.h"
#include "physics/CircleBody.h"
#include "physics/BoxShape.h"
#include "physics/DistanceJoint.h"
#include "physics/PhysicsWorld.h"
#include "physics/Constants.h"
#include "renderer/Renderer.h"

int main()
{
    sf::RenderWindow window(sf::VideoMode({800, 600}), "Physics Sandbox");

    // clock for dt
    sf::Clock clock;

    PhysicsWorld world;
    Renderer renderer(window);

    // Create Static Floor (width: 800, height: 50, pos: 400, 575 so top is at 550)
    auto floorShape = std::make_unique<Physics::BoxCollider>(800.f, 50.f);
    auto floorBody = std::make_unique<Physics::RigidBody>(Physics::Vector2(400.f, 575.f), 0.f, std::move(floorShape));
    world.addBody(std::move(floorBody));

    // Create Static Left Wall (width: 50, height: 600, pos: -25, 300)
    auto leftWallShape = std::make_unique<Physics::BoxCollider>(50.f, 600.f);
    auto leftWallBody = std::make_unique<Physics::RigidBody>(Physics::Vector2(-25.f, 300.f), 0.f, std::move(leftWallShape));
    world.addBody(std::move(leftWallBody));

    // Create Static Right Wall (width: 50, height: 600, pos: 825, 300)
    auto rightWallShape = std::make_unique<Physics::BoxCollider>(50.f, 600.f);
    auto rightWallBody = std::make_unique<Physics::RigidBody>(Physics::Vector2(825.f, 300.f), 0.f, std::move(rightWallShape));
    world.addBody(std::move(rightWallBody));

    // Create a Pendulum using DistanceJoint
    auto anchorShape = std::make_unique<Physics::BoxCollider>(20.f, 20.f);
    auto anchorBody = std::make_unique<Physics::RigidBody>(Physics::Vector2(400.f, 100.f), 0.f, std::move(anchorShape));
    Physics::RigidBody* anchorPtr = anchorBody.get();
    world.addBody(std::move(anchorBody));

    auto pendShape = std::make_unique<Physics::BoxCollider>(40.f, 40.f);
    auto pendBody = std::make_unique<Physics::RigidBody>(Physics::Vector2(550.f, 100.f), 1.f, std::move(pendShape));
    Physics::RigidBody* pendPtr = pendBody.get();
    world.addBody(std::move(pendBody));

    auto joint = std::make_unique<Physics::DistanceJoint>(anchorPtr, pendPtr, Physics::Vector2(0.f, 0.f), Physics::Vector2(0.f, 0.f), 150.f);
    world.addConstraint(std::move(joint));

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

            // Toggle debug draw with D key
            if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>())
            {
                if (keyPressed->code == sf::Keyboard::Key::D)
                {
                    renderer.setDebugDraw(!renderer.isDebugDraw());
                }
            }

            // Left-click -> spawn a circle
            if (const auto *mousepressed = event->getIf<sf::Event::MouseButtonPressed>())
            {
                sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                Physics::Vector2 spawnPos(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));

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
        renderer.drawConstraints(world.getConstraints());
        renderer.drawBodies(world.getBodies());
        window.display();
    }

    return 0;
}
