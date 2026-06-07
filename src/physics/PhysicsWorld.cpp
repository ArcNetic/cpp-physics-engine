#include "PhysicsWorld.h"
#include "Constants.h"
#include <cmath>

void PhysicsWorld::update(float dt)
{
    // Update all balls (gravity + floor collision)
    for (Ball &ball : balls)
    {
        ball.update(dt, Constants::GRAVITY, Constants::FLOOR_Y, Constants::RESTITUTION, Constants::SLEEP_THRESHOLD);
    }

    // Reset colors to default
    for (Ball &ball : balls)
    {
        ball.setColor(sf::Color::White);
    }

    // Check for collisions between all ball pairs
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

            // Guard against divide-by-zero when balls overlap exactly
            if (distance < 0.0001f)
            {
                continue;
            }

            // Collision detected
            if (distance < minDistance)
            {
                float overlap = minDistance - distance;
                sf::Vector2f collisionNormal{dx / distance, dy / distance};

                // Push balls equally apart
                sf::Vector2f correction = collisionNormal * (overlap / 2.f);

                balls[i].setPosition(posA - correction);
                balls[j].setPosition(posB + correction);

                sf::Vector2f velocityA = balls[i].getVelocity();
                sf::Vector2f velocityB = balls[j].getVelocity();

                sf::Vector2f relativeVelocity = velocityB - velocityA;

                float velocityAlongNormal = relativeVelocity.x * collisionNormal.x + relativeVelocity.y * collisionNormal.y;
                if (velocityAlongNormal > 0) // already separating
                {
                    continue;
                }

                float impulse = -(1.f + Constants::RESTITUTION) * velocityAlongNormal;
                impulse /= 2.f; // equal mass assumption

                sf::Vector2f impulseVector = collisionNormal * impulse;
                balls[i].setVelocity(velocityA - impulseVector);
                balls[j].setVelocity(velocityB + impulseVector);
            }
        }
    }
}

void PhysicsWorld::render(sf::RenderWindow &window)
{
    for (auto &ball : balls)
    {
        ball.render(window);
    }
}

void PhysicsWorld::addBall(const Ball &ball)
{
    balls.push_back(ball);
}

std::vector<Ball> &PhysicsWorld::getBalls()
{
    return balls;
}