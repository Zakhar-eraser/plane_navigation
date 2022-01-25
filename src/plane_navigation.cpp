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

void Navigator::TransformMap(Position start)
{
    for(auto &line : transformedMap)
    {
        line.second.TransformSegment(start);
    }
}

Navigator::Navigator(string configPath, SensorScans *scans)
{
    sleepTime = 1.0f / CONTROLLER_LOOP_RATE;
    this->scans = scans;
    isUpdate = false;
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

void Navigator::CalculationCycle(Position mapStart, Position startInRelated, float range,
                                 float yaw, float mapAngle, float absAngle)
{
    float s = sin(yaw);
    float c = cos(yaw);
    TransformMap(mapStart);
    Position mapStartRet = Position()- mapStart;
    for(auto &crossLine : transformedMap)
    {
        Segment &line = crossLine.second;
        if(s * line.normal.x - c * line.normal.y < -0.08f)
        {
            float n2 = line.normal.y;
            if(abs(n2) > 0.08f)
            {
                float x2 = line.start.x;
                float y2 = line.start.y;
                float m2 = line.normal.x;
                float xc = s * range;
                float yc = m2 / n2 * (x2 - xc) + y2;
                float y0 = yc + range * c;
                float rot = yaw + M_PI_2;
                Position tempPos(0, y0);
                if(!line.NotInRange(std::make_pair(xc, yc)))
                {
                    //Offset in relative frame
                    tempPos = Rotated(tempPos, -rot);
                    tempPos = Transformed(tempPos, startInRelated);
                    tempPos = Rotated(tempPos, rot);
                    //Getting a pos in the map frame
                    tempPos = Rotated(Transformed(tempPos, mapStartRet), mapAngle);
                    poses.push_back(Pose(tempPos.x, tempPos.y, absAngle));
                }
            }
        }
    }
    TransformMap(mapStartRet);
}

void Navigator::CalculatePosesByLaserPair(float absAngle, float globAbsAngle, float roll, float pitch, LaserData left, LaserData front)
{
    if(left.isOn && front.isOn)
    {
        for(auto &wall : map)
        {
            float mapAngle = wall.second.angle;
            float relYaw = absAngle - mapAngle + M_PI;
            if(cos(mapAngle) * cos(absAngle) + sin(mapAngle) * sin(absAngle) < -0.08f)
            {
                RotateMap(-mapAngle);
                Segment seg = transformedMap[wall.first];
                float frontRange, otherRange, startX, startY;

                frontRange = (front.range + front.offsets.y - left.offsets.y) * cos(pitch);
                otherRange = (left.range + front.offsets.x - left.offsets.x) * cos(roll);
                CalculationCycle(Position(seg.start.x + frontRange * cos(relYaw), seg.start.y),
                                 Position(front.offsets.x, left.offsets.y), otherRange,
                                 relYaw, mapAngle, globAbsAngle);

                RotateMap(mapAngle);
            }
        }
    }
}

void Navigator::CalculatePoses()
{
    poses.clear();
    for(auto &wall : map)
    {
        float absAngle = wall.second.angle + scans->yaw - M_PI;
        LaserData left = scans->leftLaser;
        LaserData front = scans->frontLaser;
        CalculatePosesByLaserPair(absAngle, absAngle, scans->roll, scans->pitch,
                                  left, front);
        left = scans->frontLaser;
        left.offsets = Rotated(left.offsets, M_PI_2);
        front = scans->rightLaser;
        front.offsets = Rotated(front.offsets, M_PI_2);
        CalculatePosesByLaserPair(absAngle - M_PI_2, absAngle, scans->pitch, scans->roll,
                                  left, front);
        left = scans->rightLaser;
        left.offsets = Rotated(left.offsets, M_PI);
        front = scans->backLaser;
        front.offsets = Rotated(front.offsets, M_PI);
        CalculatePosesByLaserPair(absAngle - M_PI, absAngle, scans->roll, scans->pitch,
                                  left, front);
        left = scans->backLaser;
        left.offsets = Rotated(left.offsets, -M_PI_2);
        front = scans->leftLaser;
        front.offsets = Rotated(front.offsets, -M_PI_2);
        CalculatePosesByLaserPair(absAngle + M_PI_2, absAngle, scans->pitch, scans->roll,
                                  left, front);
    }
}

Pose Navigator::GetMeanPosition()
{
    Pose meanPose;
    unsigned int count = poses.size();
    for(Pose &pose : poses)
    {
        meanPose.position = meanPose.position + pose.position;
        meanPose.angle += pose.angle;
    }
    meanPose.position.x /= count;
    meanPose.position.y /= count;
    meanPose.angle /= count;
    return meanPose;
}

void Navigator::CalibrateSymmetricMap()
{
    float realX = scans->leftLaser.range + scans->rightLaser.range - scans->leftLaser.offsets.x + scans->rightLaser.offsets.x;
    float realY = scans->frontLaser.range + scans->backLaser.range - scans->backLaser.offsets.y + scans->frontLaser.offsets.y;
    for(auto &wall : map)
    {
        Segment &seg = wall.second;
        seg.start.x = seg.start.x / abs(seg.start.x) * realX / 2;
        seg.end.x = seg.end.x / abs(seg.end.x) * realX / 2;
        seg.start.y = seg.start.y / abs(seg.start.y) * realY / 2;
        seg.end.y = seg.end.y / abs(seg.end.y) * realY / 2;
    }
}

Pose Navigator::GetMinDiversePosition(Pose initPos)
{
    return *(std::min_element(poses.begin(), poses.end(), [&initPos](Pose a, Pose b)
    {
        Position ap = a.position;
        Position bp = b.position;
        Position ip = initPos.position;
        return (ap.x - ip.x) * (ap.x - ip.x) + (ap.y - ip.y) * (ap.y - ip.y) < (bp.x - ip.x) * (bp.x - ip.x) + (bp.y - ip.y) * (bp.y - ip.y);
    }));
}