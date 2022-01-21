#include "NavigationStructs.hpp"

Position &Position::operator=(pair position)
{
    x = position.first;
    y = position.second;
    return *this;
}

Position::Position(){}

Position::Position(float x, float y)
{
    this->x = x;
    this->y = y;
}

Position::Position(pair position)
{
    this->x = position.first;
    this->y = position.second;
}

Position& Position::operator+(Position &pose)
{
    this->x += pose.x;
    this->y += pose.y;
    return *this;
}

Position& Position::operator-(Position &pose)
{
    this->x -= pose.x;
    this->y -= pose.y;
    return *this;
}

LaserData::LaserData(){}

LaserData::LaserData(float range, float x, float y, bool isOn)
{
    this->range = range;
    offsets.x = x;
    offsets.y = y;
    this->isOn = isOn;
}

LaserData::LaserData(float range, Position offsets, bool isOn)
{
    this->range = range;
    this->offsets = offsets;
    this->isOn = isOn;
}

Pose::Pose(){}

Pose::Pose(float x, float y, float angle)
{
    position.x = x;
    position.y = y;
    this->angle = angle;
}

Pose::Pose(Position position, float angle)
{
    this->position = position;
    this->angle = angle;
}