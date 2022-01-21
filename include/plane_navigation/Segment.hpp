#ifndef SEGMENT
#define SEGMENT
#include <map>
#include <cmath>
#include <limits>
#include <NavigationStructs.hpp>
#define NANF std::numeric_limits<float>::quiet_NaN()

class Navigator;

class Segment
{
    private:
        float allowance = 0.1f;
        Position start;
        Position end;
        Position normal;
        float angle;
        void TransformSegment(Position start);
        Segment RotatedSegment(float angle);
        Segment GetLineWithOffset(float offset);
        std::map<std::string, Segment> TransformedMap(std::map<std::string, Segment> &map, float angle);
        bool NotInRange(Position pos);
    public:
        Segment();
        Segment(Position point1, Position point2, float angle);
        Segment(Position point1, Position point2, Position normal);
        Position Cross(Segment segment);
        friend class Navigator;
};

Position Rotated(Position pointInRelated, float angleInWorld);
Position Transformed(Position point, Position center);
#endif