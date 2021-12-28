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

Offsets::Offsets(pair front, pair back, pair left, pair right)
{
    this->front = front;
    this->back = back;
    this->left = left;
    this->right = right;
}

Pose::Pose(){}
Offsets::Offsets(){}

Pose::Pose(float x, float y, float angle)
{
    this->x = x;
    this->y = y;
    this->angle = angle;
}

Pose::Pose(pair position)
{
    this->x = position.first;
    this->y = position.second;
}

Pose& Pose::operator+(Pose& pose)
{
    this->x += pose.x;
    this->y += pose.y;
    return *this;
}

Pose& Pose::operator-(Pose& pose)
{
    this->x -= pose.x;
    this->y -= pose.y;
    return *this;
}