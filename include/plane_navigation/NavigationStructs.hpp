#ifndef NAV_STRUCTS
#define NAV_STRUCTS

#include <utility>
using pair = std::pair<float, float>;

struct Switcher
{
    bool left;
    bool right;
    bool back;

    Switcher();
    Switcher(bool left, bool right, bool back);
};

struct Pose
{
    float x = 0.0f;
    float y = 0.0f;
    float angle = 0.0f;

    Pose();
    Pose(pair position);
    Pose(float x, float y, float angle);
    Pose& operator=(pair position);
    Pose& operator+(Pose& pose);
    Pose& operator-(Pose& pose);
};

struct Offsets
{
    Pose front;
    Pose back;
    Pose left;
    Pose right;
    Offsets();
    Offsets(pair front, pair back, pair left, pair right);
};

struct SensorScans
{
    float left = 0.0f;
    float right = 0.0f;
    float front = 0.0f;
    float back = 0.0f;
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
};
#endif