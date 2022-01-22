#ifndef PLANE_NAVIGATION
#define PLANE_NAVIGATION
#include <utility>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <thread>
#include "Segment.hpp"
using string = std::string;

class Navigator
{
private:
    bool frontUpdated;
    bool backUpdated;
    bool leftUpdated;
    bool rightUpdated;
    bool angleUpdated;
    std::thread navigatorThread;
    bool threadStop;
    float sleepTime;
    std::map<std::string, Segment> map;
    std::map<std::string, Segment> transformedMap;
    std::vector<Position> linkedPoses;
    std::vector<Pose> poses;
    SensorScans *scans = nullptr;
    void ThreadLoop();
    void CalculationCycle(Position mapStart, Position startInRelated, float range,
                                 float yaw, float mapAngle, float absAngle);
    void TransformMap(Position start);
    void RotateMap(float angle);
    void SetNavigatorState(bool stop);
    void CalibrateSymmetricMap();
    void ReturnPose(float yaw, float wallAngle, Position oldCenter);
    void CalculatePosesByLaserPair(float absAngle, float globAbsAngle, float roll, float pitch, LaserData left, LaserData front);
public:
    bool isUpdate;
    Navigator(std::string configPath, SensorScans *scans);
    ~Navigator();
    void StartNavigator();
    void CalculatePoses();
    Pose GetMinDiversePosition(Pose initPos);
    Pose GetMeanPosition();
};
#endif