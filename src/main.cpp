#include <SFML/Graphics.hpp>
#include <iostream>
#include <optional>
#include <cmath>
#include <vector>
#include "physics/Ball.h"
#include "physics/PhysicsWorld.h"

int main()
{
    sf::RenderWindow window(sf::VideoMode({800, 600}), "Physics Sandbox");

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
    // std::vector<Ball> balls;  NOT NEEDED ANYMORE

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
                    //balls.emplace_back(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y), 50.f);
                    world.addball(Ball(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y), 50.f));
                }
            }
        }

        // Update all balls
        for (Ball &ball : world.getBalls())
        {
            ball.update(dt, gravity, floorY, e, sleepThreshold);
        }

        // default color
        for (Ball &ball : world.getBalls())
        {
            ball.setColor(sf::Color::White);
        }

        // check for collisions
        for (size_t i = 0; i < world.getBalls().size(); i++)
        {
            for (size_t j = i + 1; j < world.getBalls().size(); j++)
            {
                world.update(dt);

                sf::Vector2f posA = world.getBalls()[i].getPosition();

                sf::Vector2f posB = world.getBalls()[j].getPosition();

                float radiusA = world.getBalls()[i].getRadius();

                float radiusB = world.getBalls()[j].getRadius();

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

                    world.getBalls()[i].setPosition(posA - correction);
                    world.getBalls()[j].setPosition(posB + correction);

                    sf::Vector2f velocityA = world.getBalls()[i].getVelocity();
                    sf::Vector2f velocityB = world.getBalls()[j].getVelocity();

                    sf::Vector2f relativeVelocity = velocityB - velocityA;

                    float velocityAlongNormal = relativeVelocity.x * collisionNormal.x + relativeVelocity.y * collisionNormal.y;
                    if (velocityAlongNormal > 0) // already separating
                    {
                        continue;
                    }
                    else
                    {
                        float impulse = -(1.f + e) * velocityAlongNormal;
                        impulse /= 2.f;

                        sf::Vector2f impulseVector = collisionNormal * impulse;
                        world.getBalls()[i].setVelocity(velocityA - impulseVector);
                        world.getBalls()[j].setVelocity(velocityB + impulseVector);
                    }
                }
            }
        }

        // render
        window.clear();

        window.draw(floor);
        for (Ball &ball : world.getBalls())
        {
            ball.render(window);
        }
        window.display();
    }

    return 0;
}