#ifndef NAV_STRUCTS
#define NAV_STRUCTS

#include <utility>
#include <sensor_msgs/LaserScan.h>
using pair = std::pair<float, float>;

struct Position
{
    float x = 0.0f;
    float y = 0.0f;

    Position();
    Position(pair position);
    Position(float x, float y);
    Position &operator=(pair position);
    Position operator+(Position pos);
    Position operator-(Position pos);
};

struct Pose
{
    Position position;
    float angle = 0.0f;

    Pose();
    Pose(float x, float y, float angle);
    Pose(Position position, float angle);
};

struct LaserData
{
    float range;
    Position offsets;
    bool isOn;

    LaserData();
    LaserData(float range, float x, float y, bool isOn);
    LaserData(float range, Position offsets, bool isOn);
};

struct LidarData
{
    Position offsets;
    sensor_msgs::LaserScanPtr data;
};

struct SensorScans
{
    LaserData leftLaser;
    LaserData rightLaser;
    LaserData frontLaser;
    LaserData backLaser;
    LidarData lidar;
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
};

enum LaserPair
{
    LEFT_FRONT,
    FRONT_RIGHT,
    RIGHT_BACK,
    BACK_LEFT
};
#endif