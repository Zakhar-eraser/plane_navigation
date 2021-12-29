#include "plane_navigation.hpp"
#include <numeric>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <unistd.h>

#define CONTROLLER_LOOP_RATE 30

void Navigator::TransformedMap(Segment baseLine)
{
    for(auto line : map)
    {
        transformedMap[line.first] = line.second.TransformLine(-baseLine.angle, baseLine.start);
    }
}

Navigator::Navigator(string configPath, SensorScans *scans, Switcher switcher, Offsets offsets)
{
    sleepTime = 1.0f / CONTROLLER_LOOP_RATE;
    this->scans = scans;
    isUpdate = false;
    this->switcher = switcher;
    YAML::Node node = YAML::LoadFile(configPath)["lines"];
    for(YAML::const_iterator i = node.begin(); i != node.end(); ++i)
    {
        string key = i->first.as<string>();
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
            CalculatePoses();
        }
        usleep(sleepTime * 1e6);
    }
}

void Navigator::CalculationCycle(float length, pair transform, pair laserDir)
{
    for(auto &crossLine : transformedMap)
    {
        Segment &line = crossLine.second;
        if(laserDir.first * line.normal.first + laserDir.second * line.normal.second < 0.0f)
        {
            float n2 = line.normal.second;
            if(abs(n2) > 0.08f)
            {
                float x2 = line.start.first;
                float y2 = line.start.second;
                float m2 = line.normal.first;
                float xc = transform.first * length;
                float yc = m2 / n2 * (x2 - xc) + y2; //check
                float y0 = yc - length * transform.second;
                float rot = scans->yaw + M_PI_2;
                pair relativePos(0, y0);
                if(!line.NotInRange(std::make_pair(xc, yc)))
                {
                    relativePos = Transform(relativePos, -rot);
                    relativePos.first -= laserOffsets.front.x;
                    linkedPoses.push_back(Transform(relativePos, rot));
                }
            }
        }
    }
}

void Navigator::CalculatePosesByWall(string wallId, float yaw)
{
    transformedMap.clear();
    linkedPoses.clear();
    Segment line = map[wallId];
    float c = cos(yaw);
    float s = sin(yaw);
    float offset = (scans->front - laserOffsets.front.y) * c;
    float leftRange = (scans->left + laserOffsets.front.x - laserOffsets.left.x) * cos(scans->roll);
    float rightRange = (scans->right + laserOffsets.front.x - laserOffsets.right.x) * cos(scans->roll);
    float backRange = (scans->back + laserOffsets.back.y) * cos(scans->pitch);

    pair normal = line.normal;
    Segment lineWithOffset = line.GetLineWithOffset(offset);
    TransformedMap(lineWithOffset);

    if(switcher.back) CalculationCycle(backRange, std::make_pair(c, s), std::make_pair(c, -s));
    if(switcher.left) CalculationCycle(leftRange, std::make_pair(s, -c), std::make_pair(s, -c));
    if(switcher.right) CalculationCycle(rightRange, std::make_pair(-s, c), std::make_pair(-s, c));

    pair turnedBackPosition;
    Pose turnedBackPose;
    pair offsets = lineWithOffset.start;
    for(pair &pose : linkedPoses)
    {
        turnedBackPosition = Transform(pose, lineWithOffset.angle);
        turnedBackPose.x = turnedBackPosition.first + offsets.first;
        turnedBackPose.y = turnedBackPosition.second + offsets.second;
        turnedBackPose.angle = yaw;//yaw + lineWithOffset.angle - M_PI;
        poses.push_back(turnedBackPose);
    }
}

void Navigator::CalculatePoses()
{
    poses.clear();
    for(auto &lineDescript : map)
    {
        CalculatePosesByWall(lineDescript.first, scans->yaw);
    }
}

Pose Navigator::GetMeanPosition()
{
    Pose meanPose;
    unsigned int count = poses.size();
    for(Pose &pose : poses)
    {
        meanPose = meanPose + pose;
        meanPose.angle += pose.angle;
    }
    meanPose.x /= count;
    meanPose.y /= count;
    meanPose.angle /= count;
    return meanPose;
}

void Navigator::CalibrateMap(string wallId, float absYaw)
{
    poses.clear();
    CalculatePosesByWall(wallId, absYaw);
    Pose meanPose = GetMeanPosition();
    
}

Pose Navigator::GetMinDiversePosition(Pose initPos)
{
    return *(std::min_element(poses.begin(), poses.end(), [&initPos](Pose a, Pose b)
    {
        return (a.x - initPos.x) * (a.x - initPos.x) + (a.y - initPos.y) * (a.y - initPos.y) < (b.x - initPos.x) * (b.x - initPos.x) + (b.y - initPos.y) * (b.y - initPos.y);
    }));
}