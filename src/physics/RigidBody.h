#pragma once

#include <SFML/System/Vector2.hpp>
#include <memory>
#include "Shape.h"

namespace Physics
{
    class RigidBody
    {
    public:
        RigidBody(sf::Vector2f position, float mass, std::unique_ptr<Shape> shape);

        // Physics integration (semi-implicit Euler)
        void integrate(float dt);

        // Force accumulation — cleared after each integration step
        void applyForce(const sf::Vector2f& force);

        // Accessors
        sf::Vector2f getPosition() const;
        void setPosition(const sf::Vector2f& pos);

        sf::Vector2f getVelocity() const;
        void setVelocity(const sf::Vector2f& vel);

        float getMass() const;
        float getInverseMass() const;

        float getRestitution() const;
        void setRestitution(float e);

        const Shape* getShape() const;

        float getAngle() const;
        void setAngle(float a);

        // A static body has infinite mass (inverseMass == 0) and never moves
        bool isStatic() const;

    private:
        sf::Vector2f position;
        sf::Vector2f velocity;
        sf::Vector2f forceAccumulator;

        float mass;
        float inverseMass; // 0 = static/infinite mass
        float restitution;
        float angle;

        std::unique_ptr<Shape> shape;
    };
}
