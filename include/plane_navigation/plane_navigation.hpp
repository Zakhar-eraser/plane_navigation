#ifndef PLANE_NAVIGATION
#define PLANE_NAVIGATION
#include <utility>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <thread>
#include <NavigationStructs.hpp>
#include "Segment.hpp"
using pair = std::pair<float, float>;
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
    Switcher switcher;
    std::vector<pair> linkedPoses;
    std::vector<Pose> poses;
    SensorScans *scans = nullptr;
    Offsets laserOffsets;
    void ThreadLoop();
    void CalculationCycle(bool switcher, pair mapStart, pair startInRelated, float otherRange,
                                 pair transform, pair laserDir, float yaw, float mapAngle);
    void TransformMap(pair start);
    void RotateMap(float angle);
    void SetNavigatorState(bool stop);
    void CalibrateMap(string wallId, float absYaw);
    void ReturnPose(float yaw, float wallAngle, pair oldCenter);
public:
    bool isUpdate;
    Navigator(std::string configPath, SensorScans *scans, Switcher switcher, Offsets offsets);
    ~Navigator();
    void StartNavigator();
    void CalculatePoses();
    void CalculatePosesByWall(string wallId, float yaw);
    Pose GetMinDiversePosition(Pose initPos);
    Pose GetMeanPosition();
};
#endif