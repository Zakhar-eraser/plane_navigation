#ifndef SEGMENT
#define SEGMENT
#include <utility>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <thread>
#include <NavigationStructs.hpp>

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
    void CalculationCycle(std::string passingId, float length, std::pair<float, float> transform);
    void TransformedMap(std::pair<float, float> start, float angle);
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
        std::pair<float, float> start;
        std::pair<float, float> end;
        std::pair<float, float> normal;
        Segment TransformLine(float angle, std::pair<float, float> start);
        Segment GetLineWithOffset(float offset);
        friend float GetPositionByWall(Segment wall, float distance, std::pair<float, float> vec);
        std::map<std::string, Segment> TransformedMap(std::map<std::string, Segment> &map, float angle);
        bool NotInRange(std::pair<float, float> pos);
    public:
        Segment();
        Segment(std::pair<float, float> point1, std::pair<float, float> point2, float angle);
        Segment(std::pair<float, float> point1, std::pair<float, float> point2, std::pair<float, float> normal);
        friend class Navigator;
};

float GetRotationAngle(std::pair<float, float> curNormal, std::pair<float, float> goalNormal);

std::pair<float, float> Transform(std::pair<float, float> pointInRelated, float angleInWorld);
#endif