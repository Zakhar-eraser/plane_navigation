#include "Mapper.hpp"

LinkedPoint::LinkedPoint(float x, float y)
{
    this->x = x;
    this->y = y;
}

LinkedPoint::LinkedPoint(Position pos)
{
    this->x = pos.x;
    this->y = pos.y;
}

Contour::Contour()
{
    firstPoint = new LinkedPoint(0.0f, 0.0f);
    this->lastPoint = firstPoint;
}

Contour::Contour(Position firstPoint)
{
    this->firstPoint = new LinkedPoint(firstPoint);
    this->lastPoint = this->firstPoint;
}

LinkedPoint *Contour::forEach(LinkedPoint *(*op)(LinkedPoint*), bool (*exitCondition)(LinkedPoint*))
{
    LinkedPoint *current = firstPoint;
    do
    {
        if(exitCondition(current)) break;
        op(current);
        current = current->pnextPoint;
    } while (current != firstPoint);
    return current;
}

void Contour::AddPoint(Position pos)
{
    lastPoint = new LinkedPoint(pos);
    lastPoint->pnextPoint = firstPoint;
}

void Contour::RemovePoint(Position pos)
{

}