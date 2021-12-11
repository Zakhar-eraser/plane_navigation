#ifndef SEGMENT
#define SEGMENT
#include <utility>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <plane_navigation/DroneSensors.h>
class Segment
{
    private:
        std::pair<float, float> start;
        std::pair<float, float> end;
        std::pair<float, float> normal;
    public:
        Segment();
        Segment(std::pair<float, float> point1, std::pair<float, float> point2, float angle);
        Segment(std::pair<float, float> point1, std::pair<float, float> point2, std::pair<float, float> normal);
        Segment TransformLine(float angle, std::pair<float, float> start);
        Segment GetLineWithOffset(float offset);
        friend float GetPositionByWall(Segment wall, float distance, std::pair<float, float> vec);
        std::map<std::string, Segment> TransformedMap(std::map<std::string, Segment> &map, float angle);
        bool NotInRange(std::pair<float, float> pos);
        std::pair<float, float> GetNorm();
        std::pair<float, float> GetStart();
        std::pair<float, float> GetEnd();
};

float GetRotationAngle(std::pair<float, float> curNormal, std::pair<float, float> goalNormal);

std::pair<float, float> Transform(std::pair<float, float> pointInRelated, float angleInWorld);

std::pair<float, float> GetPosition(std::map<std::string, Segment> map, std::pair<float, float> initPos,
                                    plane_navigation::DroneSensorsConstPtr scans);
#endif