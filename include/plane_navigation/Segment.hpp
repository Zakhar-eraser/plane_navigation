#ifndef SEGMENT
#define SEGMENT
#include <map>
#include <cmath>
#include <limits>
#define NANF std::numeric_limits<float>::quiet_NaN()
using pair = std::pair<float, float>;

class Navigator;

class Segment
{
    private:
        float allowance = 0.1f;
        pair start;
        pair end;
        pair normal;
        float angle;
        void TransformSegment(pair start);
        Segment RotatedSegment(float angle);
        Segment GetLineWithOffset(float offset);
        std::map<std::string, Segment> TransformedMap(std::map<std::string, Segment> &map, float angle);
        bool NotInRange(pair pos);
    public:
        Segment();
        Segment(pair point1, pair point2, float angle);
        Segment(pair point1, pair point2, pair normal);
        pair Cross(Segment segment);
        friend class Navigator;
};

pair Rotate(pair pointInRelated, float angleInWorld);
pair Transform(pair point, pair center);
#endif