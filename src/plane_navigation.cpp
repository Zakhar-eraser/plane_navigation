#include "plane_navigation.hpp"
#include <numeric>
#define NANF std::numeric_limits<float>::quiet_NaN()

float GetRotationAngle(std::pair<float, float> curNormal, std::pair<float, float> goalNormal)
{
    return asin(curNormal.first * goalNormal.second - curNormal.second * goalNormal.first);
}

std::pair<float, float> Transform(std::pair<float, float> pointInRelated, float angleInWorld)
{
    float s = sin(angleInWorld);
    float c = cos(angleInWorld);
    return std::make_pair<float, float> (pointInRelated.first * c - pointInRelated.second * s,
                                         pointInRelated.second * c + pointInRelated.first * s);
}

Segment::Segment()
{
    
}

Segment::Segment(std::pair<float, float> point1, std::pair<float, float> point2, float angle)
{
    start = point1;
    end = point2;
    this->normal = std::make_pair(cos(angle), sin(angle));
}

Segment::Segment(std::pair<float, float> point1, std::pair<float, float> point2, std::pair<float, float> normal)
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

Segment Segment::TransformLine(float angle, std::pair<float, float> start)
{
    std::pair<float, float> newStart = Transform(std::make_pair(this->start.first - start.first,
                                                              this->start.second - start.second),
                                                              angle);
    std::pair<float, float> newEnd = Transform(std::make_pair(this->end.first - start.first,
                                                              this->end.second - start.second),
                                                              angle);
    std::pair<float, float> newNormal = Transform(this->normal, angle);
    return Segment(newStart, newEnd, newNormal);
}

bool Segment::NotInRange(std::pair<float, float> pos)
{
    float minX = std::min(start.first, end.first);
    float maxX = std::max(start.first, end.first);
    float minY = std::min(start.second, end.second);
    float maxY = std::max(start.second, end.second);
    return pos.first < minX || pos.first > maxX || pos.second < minY || pos.second > maxY;
}

float GetPositionByWall(Segment wall, float distance, std::pair<float, float> vec)
{
    float x2 = wall.start.first;
    float y2 = wall.start.second;
    float m2 = wall.normal.first;
    float n2 = wall.normal.second;
    float y0;
    float yc;
    if(n2 == 0.0f)
    {
        return NANF;
    }
    else
    {
        yc = m2 / n2 * (x2 - vec.first * distance) + y2;
        y0 = yc - distance * vec.second;
    }
    if(wall.NotInRange(std::make_pair(0, yc)))
    {
        return NANF;
    } return y0;
}

std::pair<float, float> Segment::GetNorm()
{
    return normal;
}

std::pair<float, float> Segment::GetStart()
{
    return start;
}

std::pair<float, float> Segment::GetEnd()
{
    return end;
}

std::map<std::string, Segment> Segment::TransformedMap(std::map<std::string, Segment> &map, float angle)
{
    std::map<std::string, Segment> newMap;
    for(auto line : map)
    {
        newMap[line.first] = line.second.TransformLine(angle, start);
    }
    return newMap;
}

std::pair<float, float> GetPosition(std::map<std::string, Segment> map, std::pair<float, float> initPos,
                                    plane_navigation::DroneSensorsConstPtr scans)
{
    int lineCount = map.size();
    std::vector<std::pair<float, float>> poses;
    for(auto &lineDescript : map)
    {
        Segment line(lineDescript.second);
        float angle = scans->angle.data;
        float c = cos(angle);
        float s = sin(angle);
        float offset = scans->front.range * c;
        std::pair<float, float> normal = line.GetNorm();
        float turn = GetRotationAngle(std::make_pair(1.0f, 0.0f), normal);
        Segment lineWithOffset = line.GetLineWithOffset(offset);
        std::map<std::string, Segment> transformedMap = lineWithOffset.TransformedMap(map, -turn);
        std::vector<float> positions;

        int i = 0;
        float y;
        for(auto &crossLineBack : transformedMap)
        {
            if(crossLineBack.first != lineDescript.first)
            {
                y = GetPositionByWall(crossLineBack.second, scans->back.range,
                                             std::make_pair(c, s));
                if(!std::isnan(y)) positions.push_back(y);
                for(auto &crossLineLeft : transformedMap)
                {
                    y = GetPositionByWall(crossLineLeft.second, scans->left.range,
                                                 std::make_pair(s, -c));
                    if(!std::isnan(y)) positions.push_back(y);
                    for(auto &crossLineRight : transformedMap)
                    {
                        if(crossLineRight.first != crossLineLeft.first)
                        {
                            y = GetPositionByWall(crossLineRight.second, scans->right.range,
                                                         std::make_pair(-s, c));
                            if(!std::isnan(y)) positions.push_back(y);
                            i++;
                        }
                    }
                }
            }
        }

        int size;
        std::pair<float, float> turnedBackPose;
        std::pair<float, float> offsets = lineWithOffset.GetStart();
        for(auto &pose : positions)
        {
            if(size > 0)
            {
                turnedBackPose = Transform(std::make_pair(0, pose), turn);
                turnedBackPose.first += offsets.first;
                turnedBackPose.second += offsets.second;
                poses.push_back(turnedBackPose);
            }
        }
    }

    return *(std::min_element(poses.begin(), poses.end(), [initPos](std::pair<float, float> a, std::pair<float, float> b)
    {
        float x1 = a.first;
        float y1 = a.second;
        float x2 = b.first;
        float y2 = b.second;
        float x = initPos.first;
        float y = initPos.second;
        return (x1 - x) * (x1 - x) + (y1 - y) * (y1 - y) < (x2 - x) * (x2 - x) + (y2 - y) * (y2 - y);
    }));
}