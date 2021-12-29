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

pair Segment::Cross(Segment segment)
{
    if(sin(this->angle - segment.angle) > 0.08)
    {
        float m1 = normal.first;
        float n1 = normal.second;
        float m2 = segment.normal.first;
        float n2 = segment.normal.second;
        float x1 = start.first;
        float y1 = start.second;
        float x2 = segment.start.first;
        float y2 = segment.start.second;
        pair cross;
        if(abs(n2) > 0.08)
        {
            if(abs(m1) > 0.08)
            {
                cross.second = m2 / n2 * (x2 - n1 / m1 * y1 - x1) / (1 - m2 * n1 / (n2 * m1));
            }
            else
            {
                cross.second = y1;
            }
            cross.first = n2 / m2 * (y2 - cross.second) + x2;
        }
        else
        {
            cross.first = x2;
            cross.second = m1 / n1 * (x1 - cross.first) + y1;
        }
        if(!segment.NotInRange(cross)) return cross;
    }
    return std::make_pair(NANF, NANF);
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
    float minX = std::min(start.first, end.first) - allowance;
    float maxX = std::max(start.first, end.first) + allowance;
    float minY = std::min(start.second, end.second) - allowance;
    float maxY = std::max(start.second, end.second) + allowance;
    return pos.first < minX || pos.first > maxX || pos.second < minY || pos.second > maxY;
}