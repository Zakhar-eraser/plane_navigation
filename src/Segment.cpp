#include "Segment.hpp"

pair Transform(pair pointInRelated, float angleInWorld)
{
    float s = sin(angleInWorld);
    float c = cos(angleInWorld);
    return std::make_pair<float, float> (pointInRelated.first * c - pointInRelated.second * s,
                                         pointInRelated.second * c + pointInRelated.first * s);
}

Segment::Segment()
{
    
}

Segment::Segment(pair point1, pair point2, float angle)
{
    start = point1;
    end = point2;
    this->normal = std::make_pair(cos(angle), sin(angle));
    this->angle = angle;
}

Segment::Segment(pair point1, pair point2, pair normal)
{
    start = point1;
    end = point2;
    this->normal = normal;
}

Segment Segment::GetLineWithOffset(float offset)
{
    Segment line(*this);
    float xOffset = normal.first * offset;
    float yOffset = normal.second * offset;
    line.start = std::make_pair(start.first + xOffset, start.second + yOffset);
    line.end = std::make_pair(end.first + xOffset, end.second + yOffset);
    return line;
}

Segment Segment::TransformLine(float angle, pair start)
{
    pair newStart = Transform(std::make_pair(this->start.first - start.first,
                                                              this->start.second - start.second),
                                                              angle);
    pair newEnd = Transform(std::make_pair(this->end.first - start.first,
                                                              this->end.second - start.second),
                                                              angle);
    pair newNormal = Transform(this->normal, angle);
    return Segment(newStart, newEnd, newNormal);
}

bool Segment::NotInRange(pair pos)
{
    float minX = std::min(start.first, end.first);
    float maxX = std::max(start.first, end.first);
    float minY = std::min(start.second, end.second);
    float maxY = std::max(start.second, end.second);
    return pos.first < minX || pos.first > maxX || pos.second < minY || pos.second > maxY;
}