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
    std::thread navigatorThread;
    bool threadStop;
    float sleepTime;
    std::map<std::string, Segment> map;
    std::map<std::string, Segment> transformedMap;
    std::map<unsigned int, Pose> poses;
    std::vector<Pose> lastPoses;
    std::vector<Pose> pairPoses;
    SensorScans *scans = nullptr;
    Pose lastPose;
    void ThreadLoop();
    void CalculationCycle(Position mapStart, Position startInRelated, float range,
                                 float yaw, float mapAngle, float absAngle);
    //Filters
    Pose GetMinDiversePose(Pose initPos);
    Pose GetSlowestPose(Pose lastPose);
    Pose GetMeanPosition();
    //Transforms
    void TransformMap(Position start);
    void RotateMap(float angle);

    void SetNavigatorState(bool stop);
    void CalibrateSymmetricMap();
    void CalculatePosesByLaserPair(float axisDir, unsigned int pair, float roll, float pitch, LaserData left, LaserData front);
public:
    bool isUpdate;
    Navigator(std::string configPath, SensorScans *scans);
    ~Navigator();
    void StartNavigator();
    void CalculatePose();
    void SetLastPose(Pose pose);
    Pose GetPose();
};
#endif