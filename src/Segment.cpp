#include "Segment.hpp"

Position Rotated(Position pointInRelated, float angleInWorld)
{
    float s = sin(angleInWorld);
    float c = cos(angleInWorld);
    return Position(pointInRelated.x * c - pointInRelated.y * s,
                                         pointInRelated.y * c + pointInRelated.x * s);
}

Position Transformed(Position point, Position center)
{
    return Position(point.x - center.x, point.y - center.y);
}

Segment::Segment()
{
    
}

Segment::Segment(Position point1, Position point2, float angle)
{
    start = point1;
    end = point2;
    this->normal = Position(cos(angle), sin(angle));
    this->angle = angle;
}

Segment::Segment(Position point1, Position point2, Position normal)
{
    start = point1;
    end = point2;
    this->normal = normal;
}

Position Segment::Cross(Segment segment)
{
    if(sin(this->angle - segment.angle) > 0.08)
    {
        float m1 = normal.x;
        float n1 = normal.y;
        float m2 = segment.normal.x;
        float n2 = segment.normal.y;
        float x1 = start.x;
        float y1 = start.y;
        float x2 = segment.start.x;
        float y2 = segment.start.y;
        Position cross;
        if(abs(n2) > 0.08)
        {
            if(abs(m1) > 0.08)
            {
                cross.y = m2 / n2 * (x2 - n1 / m1 * y1 - x1) / (1 - m2 * n1 / (n2 * m1));
            }
            else
            {
                cross.y = y1;
            }
            cross.x = n2 / m2 * (y2 - cross.y) + x2;
        }
        else
        {
            cross.x = x2;
            cross.y = m1 / n1 * (x1 - cross.x) + y1;
        }
        if(!segment.NotInRange(cross)) return cross;
    }
    return Position(NANF, NANF);
}

Segment Segment::GetLineWithOffset(float offset)
{
    Segment line(*this);
    float xOffset = normal.x * offset;
    float yOffset = normal.y * offset;
    line.start = std::make_pair(start.x + xOffset, start.y + yOffset);
    line.end = std::make_pair(end.x + xOffset, end.y + yOffset);
    return line;
}

Segment Segment::RotatedSegment(float angle)
{
    Position newStart = Rotated(Position(this->start.x, this->start.y), angle);
    Position newEnd = Rotated(Position(this->end.x, this->end.y), angle);
    Position newNormal = Rotated(this->normal, angle);
    return Segment(newStart, newEnd, newNormal);
}

void Segment::TransformSegment(Position start)
{
    this->start = Transformed(this->start, start);
    this->end = Transformed(this->end, start);
}

bool Segment::NotInRange(Position pos)
{
    float minX = std::min(start.x, end.x) - allowance;
    float maxX = std::max(start.x, end.x) + allowance;
    float minY = std::min(start.y, end.y) - allowance;
    float maxY = std::max(start.y, end.y) + allowance;
    return pos.x < minX || pos.x > maxX || pos.y < minY || pos.y > maxY;
}