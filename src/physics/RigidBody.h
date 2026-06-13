#pragma once
#include "Vector2.h"

#include <memory>
#include "Shape.h"

namespace Physics
{
    class RigidBody
    {
    public:
        RigidBody(Vector2 position, float mass, std::unique_ptr<Shape> shape);

        // Physics integration (semi-implicit Euler)
        void integrate(float dt);

        // Force accumulation — cleared after each integration step
        void applyForce(const Vector2& force);
        void applyTorque(float torque);

        // Accessors
        Vector2 getPosition() const;
        void setPosition(const Vector2& pos);

        Vector2 getVelocity() const;
        void setVelocity(const Vector2& vel);

        float getAngularVelocity() const;
        void setAngularVelocity(float w);

        float getMass() const;
        float getInverseMass() const;

        float getMomentOfInertia() const;
        float getInverseInertia() const;

        float getRestitution() const;
        void setRestitution(float e);

        const Shape* getShape() const;

        float getAngle() const;
        void setAngle(float a);

        // A static body has infinite mass (inverseMass == 0) and never moves
        bool isStatic() const;

    private:
        Vector2 position;
        Vector2 velocity;
        Vector2 forceAccumulator;

        float angle;
        float angularVelocity;
        float torqueAccumulator;

        float mass;
        float inverseMass; // 0 = static/infinite mass
        float momentOfInertia;
        float inverseInertia; // 0 = static/infinite inertia

        float restitution;

        std::unique_ptr<Shape> shape;
    };
}

