#ifndef SEGMENT
#define SEGMENT
#include <utility>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <thread>
#include <NavigationStructs.hpp>
using pair = std::pair<float, float>;

class Segment;

class Navigator
{
private:
    std::thread navigatorThread;
    bool threadStop;
    float sleepTime;
    std::map<std::string, Segment> map;
    std::map<std::string, Segment> transformedMap;

    std::vector<float> linkedPoses;
    std::vector<Pose> poses;
    SensorScans *scans = nullptr;
    void ThreadLoop();
    void CalculationCycle(std::string passingId, float length, pair transform, pair laserDir);
    void TransformedMap(pair start, float angle);
    void SetNavigatorState(bool stop);
public:
    bool isUpdate;
    Navigator(std::string configPath, SensorScans *scans);
    ~Navigator();
    void StartNavigator();
    void CalculatePose();
    Pose GetMinDiversePosition(Pose initPos);
};

class Segment
{
    private:
        pair start;
        pair end;
        pair normal;
        Segment TransformLine(float angle, pair start);
        Segment GetLineWithOffset(float offset);
        friend float GetPositionByWall(Segment wall, float distance, pair vec);
        std::map<std::string, Segment> TransformedMap(std::map<std::string, Segment> &map, float angle);
        bool NotInRange(pair pos);
    public:
        Segment();
        Segment(pair point1, pair point2, float angle);
        Segment(pair point1, pair point2, pair normal);
        friend class Navigator;
};

float GetRotationAngle(pair curNormal, pair goalNormal);

pair Transform(pair pointInRelated, float angleInWorld);
#endif