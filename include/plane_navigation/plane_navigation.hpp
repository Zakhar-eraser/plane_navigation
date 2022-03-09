#ifndef PLANE_NAVIGATION
#define PLANE_NAVIGATION
#include <utility>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include "Segment.hpp"
#include "Filters.hpp"
#include <fstream>
using string = std::string;

class Navigator
{
private:
    //Constants
    const float angAllow = 0.09f;
    const float speedAllow = 0.15f;

    std::string configPath;
    std::map<std::string, Segment> map;
    std::map<std::string, Segment> transformedMap;
    std::map<unsigned int, Pose> poses;
    std::vector<Pose> lastPoses = std::vector<Pose>(4);
    std::vector<Pose> pairPoses;
    SensorScans *scans = nullptr;
    Pose lastPose;
    //Filters
    Pose GetMinDiversePose(Pose initPos);
    Pose GetSlowestPose(Pose lastPose);
    Pose GetMeanPosition();
    //Transforms
    void TransformMap(Position start);
    void RotateMap(float angle);
    void WriteMap();
    void WriteYAMLMap();
    void CreateRectangleMap(float wallAngle, float halfWidth, float halfLength);

public:
    Navigator(string configPath, SensorScans *scans);
    ~Navigator();
    void CalibrateWidthLength(float wallAngle);
    void CalibrateWidth(string wallId);
    void CalculatePosesByFrontWall(float axisDir, float wallAngle, string frontWallId,
                                          LaserData rotatedLeft, LaserData rotatedFront, float roll, float pitch);
    void CalculatePosesByAngleWall(float axisDir, float wallAngle, LaserData rotatedLeft, LaserData rotatedFront, float roll, float pitch);
    void CalculatePoseByLaserPair(float axisDir, unsigned int pair, float roll, float pitch, LaserData left, LaserData front);
    void CalculatePoseByLidar();
    void StartNavigator();
    void CalculatePoseByRangefinders();
    void SetLastPose(Pose pose);
    Pose GetPose();
};
#endif