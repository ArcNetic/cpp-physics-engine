#include "PhysicsWorld.h"

void PhysicsWorld::update(float dt)
{

}

void PhysicsWorld::addball(const Ball& ball)
{
    balls.push_back(ball);
}

std::vector<Ball>& PhysicsWorld::getBalls()
{
    return balls;
}