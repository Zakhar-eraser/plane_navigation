#include "NavigationStructs.hpp"

Switcher::Switcher(){}

Switcher::Switcher(bool left, bool right, bool back)
{
    this->back = back;
    this->left = left;
    this->right = right;
}

Pose &Pose::operator=(pair position)
{
    x = position.first;
    y = position.second;
    return *this;
}

Pose::Pose(){}

Pose::Pose(float x, float y, float angle)
{
    this->x = x;
    this->y = y;
    this->angle = angle;
}