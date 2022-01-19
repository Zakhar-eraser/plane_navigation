#include "plane_navigation.hpp"
#include <numeric>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <unistd.h>
#include <iostream>

#define CONTROLLER_LOOP_RATE 30

void Navigator::RotateMap(float angle)
{
    for(auto &line : map)
    {
        transformedMap[line.first] = line.second.RotatedSegment(angle);
    }
}

void Navigator::TransformMap(pair start)
{
    for(auto &line : transformedMap)
    {
        line.second.TransformSegment(start);
    }
}

Navigator::Navigator(string configPath, SensorScans *scans, Switcher switcher, Offsets offsets)
{
    laserOffsets = offsets;
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
        //if (isUpdate)
        //{
        //    CalculatePoses();
        //}
        usleep(sleepTime * 1e6);
    }
}

void Navigator::CalculationCycle(bool switcher, pair mapStart, pair startInRelated, float otherRange,
                                 pair transform, pair laserDir, float yaw, float mapAngle)
{
    if(switcher)
    {
        TransformMap(mapStart);
        pair mapStartRet(-mapStart.first, -mapStart.second);
        for(auto &crossLine : transformedMap)
        {
            Segment &line = crossLine.second;
            if(laserDir.first * line.normal.first + laserDir.second * line.normal.second < -0.08f)
            {
                float n2 = line.normal.second;
                if(abs(n2) > 0.08f)
                {
                    float x2 = line.start.first;
                    float y2 = line.start.second;
                    float m2 = line.normal.first;
                    float xc = transform.first * otherRange;
                    float yc = m2 / n2 * (x2 - xc) + y2;
                    float y0 = yc - otherRange * transform.second;
                    float rot = yaw + M_PI_2;
                    pair tempPos(0, y0);
                    if(!line.NotInRange(std::make_pair(xc, yc)))
                    {
                        //Offset in relative frame
                        tempPos = Rotate(tempPos, -rot);
                        tempPos = Transform(tempPos, startInRelated);
                        tempPos = Rotate(tempPos, rot);
                        //Getting a pos in the map frame
                        tempPos = Rotate(Transform(tempPos, mapStartRet), mapAngle);
                        poses.push_back(Pose(tempPos.first, tempPos.second, yaw + mapAngle + M_PI_2));
                    }
                }
            }
        }
        TransformMap(mapStartRet);
    }
}

void Navigator::CalculatePosesByWall(string wallId, float yaw)
{
    float c = cos(yaw);
    float s = sin(yaw);
    float mapAngle = map[wallId].angle;
    RotateMap(-mapAngle);
    Segment seg = transformedMap[wallId];
    float frontRange, otherRange, startX, startY;

    frontRange = (scans->front + laserOffsets.front.y) * cos(scans->pitch);
    otherRange = (scans->back - laserOffsets.back.y) * cos(scans->pitch);
    CalculationCycle(switcher.back, std::make_pair(seg.start.first + frontRange * cos(yaw), seg.start.second),
                     std::make_pair((laserOffsets.front.x + laserOffsets.back.x) / 2, 0), otherRange,
                     std::make_pair(c, s), std::make_pair(c, -s), yaw, mapAngle);
    
    frontRange = (scans->front + laserOffsets.front.y - laserOffsets.left.y) * cos(scans->pitch);
    otherRange = (scans->left + laserOffsets.front.x - laserOffsets.left.x) * cos(scans->roll);
    CalculationCycle(switcher.left, std::make_pair(seg.start.first + frontRange * cos(yaw), seg.start.second),
                     std::make_pair(laserOffsets.front.x, laserOffsets.left.y), otherRange,
                     std::make_pair(s, -c), std::make_pair(s, -c), yaw, mapAngle);
    
    frontRange = (scans->front + laserOffsets.front.y - laserOffsets.right.y) * cos(scans->pitch);
    otherRange = (scans->right + laserOffsets.right.x - laserOffsets.front.x) * cos(scans->roll);
    CalculationCycle(switcher.right, std::make_pair(seg.start.first + frontRange * cos(yaw), seg.start.second),
                     std::make_pair(laserOffsets.front.x, laserOffsets.right.y), otherRange,
                     std::make_pair(-s, c), std::make_pair(-s, c), yaw, mapAngle);
    RotateMap(mapAngle);
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