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

class Navigator
{
private:
    std::thread navigatorThread;
    bool threadStop;
    float sleepTime;
    std::map<std::string, Segment> map;
    std::map<std::string, Segment> transformedMap;
    Switcher switcher;
    std::vector<float> linkedPoses;
    std::vector<Pose> poses;
    SensorScans *scans = nullptr;
    void ThreadLoop();
    void CalculationCycle(float length, pair transform, pair laserDir);
    void TransformedMap(pair start, float angle);
    void SetNavigatorState(bool stop);
    void CalibrateMap(std::string wallId);
public:
    bool isUpdate;
    Navigator(std::string configPath, SensorScans *scans, Switcher switcher);
    ~Navigator();
    void StartNavigator();
    void CalculatePose();
    Pose GetMinDiversePosition(Pose initPos);
};

float GetRotationAngle(pair curNormal, pair goalNormal);

pair Transform(pair pointInRelated, float angleInWorld);
#endif