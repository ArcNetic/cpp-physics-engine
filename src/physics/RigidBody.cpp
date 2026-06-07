#include "RigidBody.h"
#include <cmath>

namespace Physics
{
    RigidBody::RigidBody(sf::Vector2f position, float mass, std::unique_ptr<Shape> shape)
        : position(position),
          velocity({0.f, 0.f}),
          forceAccumulator({0.f, 0.f}),
          angle(0.f),
          angularVelocity(0.f),
          torqueAccumulator(0.f),
          mass(mass),
          inverseMass(mass > 0.f ? 1.f / mass : 0.f),
          restitution(0.7f),
          shape(std::move(shape))
    {
        if (mass > 0.f && this->shape)
        {
            momentOfInertia = this->shape->computeMomentOfInertia(mass);
            inverseInertia = momentOfInertia > 0.f ? 1.f / momentOfInertia : 0.f;
        }
        else
        {
            momentOfInertia = 0.f;
            inverseInertia = 0.f;
        }
    }

    void RigidBody::integrate(float dt)
    {
        if (isStatic())
            return;

        // Semi-implicit Euler: update velocity first, then position
        sf::Vector2f acceleration = forceAccumulator * inverseMass;
        velocity += acceleration * dt;
        position += velocity * dt;

        // Angular integration
        float angularAcceleration = torqueAccumulator * inverseInertia;
        angularVelocity += angularAcceleration * dt;
        angle += angularVelocity * dt;

        // Apply damping
        float damping = std::pow(0.5f, dt); // Half-life of 1 second
        velocity *= damping;
        angularVelocity *= damping;

        // Clear accumulated forces and torques for the next frame
        forceAccumulator = {0.f, 0.f};
        torqueAccumulator = 0.f;
    }

    void RigidBody::applyForce(const sf::Vector2f& force)
    {
        forceAccumulator += force;
    }

    void RigidBody::applyTorque(float torque)
    {
        torqueAccumulator += torque;
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

    float RigidBody::getAngularVelocity() const
    {
        return angularVelocity;
    }

    void RigidBody::setAngularVelocity(float w)
    {
        angularVelocity = w;
    }

    float RigidBody::getMass() const
    {
        return mass;
    }

    float RigidBody::getInverseMass() const
    {
        return inverseMass;
    }

    float RigidBody::getMomentOfInertia() const
    {
        return momentOfInertia;
    }

    float RigidBody::getInverseInertia() const
    {
        return inverseInertia;
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

    float RigidBody::getAngle() const
    {
        return angle;
    }

    void RigidBody::setAngle(float a)
    {
        angle = a;
    }

    bool RigidBody::isStatic() const
    {
        return inverseMass == 0.f;
    }
}
