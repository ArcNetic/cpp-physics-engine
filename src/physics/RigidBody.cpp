#include "RigidBody.h"

namespace Physics
{
    RigidBody::RigidBody(sf::Vector2f position, float mass, std::unique_ptr<Shape> shape)
        : position(position),
          velocity({0.f, 0.f}),
          forceAccumulator({0.f, 0.f}),
          mass(mass),
          inverseMass(mass > 0.f ? 1.f / mass : 0.f),
          restitution(0.7f),
          shape(std::move(shape))
    {
    }

    void RigidBody::integrate(float dt)
    {
        if (isStatic())
            return;

        // Semi-implicit Euler: update velocity first, then position
        sf::Vector2f acceleration = forceAccumulator * inverseMass;
        velocity += acceleration * dt;
        position += velocity * dt;

        // Clear accumulated forces for the next frame
        forceAccumulator = {0.f, 0.f};
    }

    void RigidBody::applyForce(const sf::Vector2f& force)
    {
        forceAccumulator += force;
    }

    sf::Vector2f RigidBody::getPosition() const
    {
        return position;
    }

    void RigidBody::setPosition(const sf::Vector2f& pos)
    {
        position = pos;
    }

    sf::Vector2f RigidBody::getVelocity() const
    {
        return velocity;
    }

    void RigidBody::setVelocity(const sf::Vector2f& vel)
    {
        velocity = vel;
    }

    float RigidBody::getMass() const
    {
        return mass;
    }

    float RigidBody::getInverseMass() const
    {
        return inverseMass;
    }

    float RigidBody::getRestitution() const
    {
        return restitution;
    }

    void RigidBody::setRestitution(float e)
    {
        restitution = e;
    }

    const Shape* RigidBody::getShape() const
    {
        return shape.get();
    }

    bool RigidBody::isStatic() const
    {
        return inverseMass == 0.f;
    }
}
