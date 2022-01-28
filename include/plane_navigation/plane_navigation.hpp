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
    //Filters
    Pose GetMinDiversePose(Pose initPos);
    Pose GetSlowestPose(Pose lastPose);
    Pose GetMeanPosition();
    //Transforms
    void TransformMap(Position start);
    void RotateMap(float angle);

    void SetNavigatorState(bool stop);
    void CalibrateSymmetricMap();
    void CalculatePosesByFrontWall(float axisDir, float wallAngle, string frontWallId,
                                          LaserData rotatedLeft, LaserData rotatedFront, float roll, float pitch);
    void CalculatePosesByAngleWall(float axisDir, float wallAngle, LaserData rotatedLeft, LaserData rotatedFront, float roll, float pitch);
    void CalculatePoseByLaserPair(float axisDir, unsigned int pair, float roll, float pitch, LaserData left, LaserData front);
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