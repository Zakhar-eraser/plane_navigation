#include "plane_navigation.hpp"
#include <numeric>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <unistd.h>

#define CONTROLLER_LOOP_RATE 30

#define NANF std::numeric_limits<float>::quiet_NaN()

Pose::Pose(){}

Pose::Pose(float x, float y, float angle)
{
    this->x = x;
    this->y = y;
    this->angle = angle;
}

float GetRotationAngle(pair curNormal, pair goalNormal)
{
    return asin(curNormal.first * goalNormal.second - curNormal.second * goalNormal.first);
}

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

float GetPositionByWall(Segment wall, float distance, pair vec)
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

void Navigator::TransformedMap(pair start, float angle)
{
    for(auto line : map)
    {
        transformedMap[line.first] = line.second.TransformLine(angle, start);
    }
}

Navigator::Navigator(std::string configPath, SensorScans *scans)
{
    sleepTime = 1.0f / CONTROLLER_LOOP_RATE;
    this->scans = scans;
    isUpdate = false;

    YAML::Node node = YAML::LoadFile(configPath)["lines"];
    for(YAML::const_iterator i = node.begin(); i != node.end(); ++i)
    {
        std::string key = i->first.as<std::string>();
        pair start = i->second["start"].as<pair>();
        pair end = i->second["end"].as<pair>();
        float angle = i->second["angle"].as<float>();
        map[key] = Segment(start, end, angle);
    }
}

Navigator::~Navigator()
{
    threadStop = true;
    navigatorThread.join();
}

void Navigator::StartNavigator()
{
    navigatorThread = std::thread(&Navigator::ThreadLoop, this);
}

void Navigator::SetNavigatorState(bool stop)
{
    threadStop = stop;
}

void Navigator::ThreadLoop()
{
    while(!threadStop)
    {
        if (isUpdate)
        {
            CalculatePose();
        }
        usleep(sleepTime * 1e6);
    }
}

void Navigator::CalculationCycle(std::string passingId, float length, pair transform, pair laserDir)
{
    for(auto &crossLine : transformedMap)
    {
        Segment &line = crossLine.second;
        if(crossLine.first != passingId && laserDir.first * line.normal.first + laserDir.second * line.normal.second < 0.0001)
        {
            float y = GetPositionByWall(line, length,
                                         transform);
            if(!std::isnan(y)) linkedPoses.push_back(y);
        }
    }
}

void Navigator::CalculatePose()
{
    poses.clear();
    for(auto &lineDescript : map)
    {
        transformedMap.clear();
        linkedPoses.clear();
        Segment line(lineDescript.second);
        float angle = scans->angle;
        float c = cos(angle);
        float s = sin(angle);
        float offset = scans->front * c;
        pair normal = line.normal;
        float turn = GetRotationAngle(std::make_pair(1.0f, 0.0f), normal);
        Segment lineWithOffset = line.GetLineWithOffset(offset);
        TransformedMap(lineWithOffset.start, -turn);

        //CalculationCycle(lineDescript.first, scans->back, std::make_pair(c, s), std::make_pair(c, -s));
        //CalculationCycle("", scans->left, std::make_pair(s, -c), Transform(std::make_pair(-c, -s), -turn));
        //CalculationCycle("", scans->right, std::make_pair(-s, c), Transform(std::make_pair(c, s), -turn));
        CalculationCycle("", scans->left, std::make_pair(s, -c), std::make_pair(s, -c));
        CalculationCycle("", scans->right, std::make_pair(-s, c), std::make_pair(-s, c));

        pair turnedBackPosition;
        Pose turnedBackPose;
        pair offsets = lineWithOffset.start;
        for(float &pose : linkedPoses)
        {
            turnedBackPosition = Transform(std::make_pair(0, pose), turn);
            turnedBackPose.x = turnedBackPosition.first + offsets.first;
            turnedBackPose.y = turnedBackPosition.second + offsets.second;
            turnedBackPose.angle = angle + atan2(-normal.second, -normal.first);
            poses.push_back(turnedBackPose);
        }
        int ii = 0;
    }
}

Pose Navigator::GetMinDiversePosition(Pose initPos)
{
    // int ll =0;
    return *(std::min_element(poses.begin(), poses.end(), [&initPos](Pose a, Pose b)
    {
        return (a.x - initPos.x) * (a.x - initPos.x) + (a.y - initPos.y) * (a.y - initPos.y) < (b.x - initPos.x) * (b.x - initPos.x) + (b.y - initPos.y) * (b.y - initPos.y);
    }));
}