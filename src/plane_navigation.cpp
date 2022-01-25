#include "plane_navigation.hpp"
#include <numeric>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <unistd.h>
#include <iostream>

#define CONTROLLER_LOOP_RATE 30

void Navigator::SetLastPose(Pose pose)
{
    lastPose = pose;
    lastPoses[0] = lastPoses[1] = lastPoses[2] = lastPoses[3] = pose;
}

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
    lastPoses.reserve(4);
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

Pose Navigator::GetPose()
{
    return lastPose;
}

Pose Navigator::GetSlowestPose(Pose lastPose)
{
    float lastDif = 1000000;
    float dif;
    Pose slowestPose = lastPose;
    Position d1;
    Position d2;
    for(auto &pose : poses)
    {
        d1 = pose.second.position - lastPoses[pose.first].position;
        d2 = pose.second.position - lastPose.position;
        dif = (d1.x * d1.x + d1.y * d1.y) + (d2.x * d2.x + d2.y * d2.y);
        if(dif < lastDif)
        {
            lastDif = dif;
            slowestPose = pose.second;
        }
    }
    return slowestPose;
}

Pose Navigator::GetMinDiversePose(Pose lastPose)
{
    return *(std::min_element(pairPoses.begin(), pairPoses.end(), [&lastPose](Pose a, Pose b)
    {
        Position ap = a.position;
        Position bp = b.position;
        Position ip = lastPose.position;
        return (ap.x - ip.x) * (ap.x - ip.x) + (ap.y - ip.y) * (ap.y - ip.y) <
               (bp.x - ip.x) * (bp.x - ip.x) + (bp.y - ip.y) * (bp.y - ip.y);
    }));
}

void Navigator::CalculationCycle(Position mapStart, Position startInRelated, float range,
                                 float yaw, float mapAngle, float absAngle)
{
    float s = sin(yaw);
    float c = cos(yaw);
    TransformMap(mapStart);
    Position mapStartRet = Position(0, 0)- mapStart;
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
                    pairPoses.push_back(Pose(tempPos.x, tempPos.y, absAngle));
                }
            }
        }
    }
    TransformMap(mapStartRet);
}

void Navigator::CalculatePosesByLaserPair(float axisDir, unsigned int pair,
                                          float roll, float pitch, LaserData left, LaserData front)
{
    pairPoses.clear();
    if(left.isOn && front.isOn)
    {
        left.offsets = Rotated(left.offsets, -axisDir);
        front.offsets = Rotated(front.offsets,  -axisDir);
        for(auto &angleWall : map)
        {
            float angleWallAngle = angleWall.second.angle;
            float absAngle = angleWallAngle - M_PI + scans->yaw;
            float curDirAbsAngle = absAngle + axisDir; 
            for(auto &wall : map)
            {
                float mapAngle = wall.second.angle;
                if(cos(mapAngle) * cos(curDirAbsAngle) + sin(mapAngle) * sin(curDirAbsAngle) < -0.08f)
                {
                    float relYaw = curDirAbsAngle - mapAngle + M_PI;
                    RotateMap(-mapAngle);
                    Segment seg = transformedMap[wall.first];
                    float frontRange, otherRange, startX, startY;

                    frontRange = (front.range + front.offsets.y - left.offsets.y) * cos(pitch);
                    otherRange = (left.range + front.offsets.x - left.offsets.x) * cos(roll);
                    CalculationCycle(Position(seg.start.x + frontRange * cos(scans->yaw), seg.start.y),
                                     Position(front.offsets.x, left.offsets.y), otherRange,
                                     relYaw, mapAngle, absAngle);

                    RotateMap(mapAngle);
                }
            }
        }
        if(pairPoses.size() > 0)
        {
            poses[pair] = GetMinDiversePose(lastPose);
        }
    }
}

void Navigator::CalculatePose()
{
    poses.clear();
    CalculatePosesByLaserPair(0, LEFT_FRONT, scans->roll, scans->pitch,
                              scans->leftLaser, scans->frontLaser);
    CalculatePosesByLaserPair(-M_PI_2, FRONT_RIGHT, scans->pitch, scans->roll,
                              scans->frontLaser, scans->rightLaser);
    CalculatePosesByLaserPair(-M_PI, RIGHT_BACK, scans->roll, scans->pitch,
                              scans->rightLaser, scans->backLaser);
    CalculatePosesByLaserPair(M_PI_2, BACK_LEFT, scans->pitch, scans->roll,
                              scans->backLaser, scans->leftLaser);
    lastPose = GetSlowestPose(lastPose);

    for(auto &pose : poses)
    {
        lastPoses[pose.first] = pose.second;
    }
}

Pose Navigator::GetMeanPosition()
{
    Pose meanPose;
    //unsigned int count = poses.size();
    //for(Pose &pose : poses)
    //{
    //    meanPose.position = meanPose.position + pose.position;
    //    meanPose.angle += pose.angle;
    //}
    //meanPose.position.x /= count;
    //meanPose.position.y /= count;
    //meanPose.angle /= count;
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