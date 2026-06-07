#pragma once
#include <vector>
#include "Ball.h"

class PhysicsWorld
{
    
private:
    std::vector<Ball> balls;

public:
    void update(float dt);

    void addball(const Ball& ball);

    std::vector<Ball>& getBalls();
};