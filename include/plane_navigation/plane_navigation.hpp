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
    void CalculationCycle(float length, pair transform, pair laserDir);
    void TransformedMap(Segment baseLine);
    void SetNavigatorState(bool stop);
    void CalibrateMap(string wallId, float absYaw);
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

pair Transform(pair pointInRelated, float angleInWorld);
#endif