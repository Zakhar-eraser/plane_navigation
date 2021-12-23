#include "plane_navigation.hpp"
#include <numeric>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <unistd.h>

#define CONTROLLER_LOOP_RATE 30

float GetRotationAngle(pair curNormal, pair goalNormal)
{
    return asin(curNormal.first * goalNormal.second - curNormal.second * goalNormal.first);
}

void Navigator::TransformedMap(pair start, float angle)
{
    for(auto line : map)
    {
        transformedMap[line.first] = line.second.TransformLine(angle, start);
    }
}

Navigator::Navigator(std::string configPath, SensorScans *scans, Switcher switcher)
{
    sleepTime = 1.0f / CONTROLLER_LOOP_RATE;
    this->scans = scans;
    isUpdate = false;
    this->switcher = switcher;
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

void Navigator::CalculationCycle(float length, pair transform, pair laserDir)
{
    for(auto &crossLine : transformedMap)
    {
        Segment &line = crossLine.second;
        if(/*laserDir.first * line.normal.first + laserDir.second * line.normal.second < 0.0f*/true)
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

        if(switcher.back) CalculationCycle(scans->back, std::make_pair(c, s), std::make_pair(c, -s));
        if(switcher.left) CalculationCycle(scans->left, std::make_pair(s, -c), std::make_pair(s, -c));
        if(switcher.right) CalculationCycle(scans->right, std::make_pair(-s, c), std::make_pair(-s, c));

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
    return *(std::min_element(poses.begin(), poses.end(), [&initPos](Pose a, Pose b)
    {
        return (a.x - initPos.x) * (a.x - initPos.x) + (a.y - initPos.y) * (a.y - initPos.y) < (b.x - initPos.x) * (b.x - initPos.x) + (b.y - initPos.y) * (b.y - initPos.y);
    }));
}