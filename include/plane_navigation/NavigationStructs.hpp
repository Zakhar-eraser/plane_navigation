#ifndef NAV_STRUCTS
#define NAV_STRUCTS
struct Pose
{
    float x = 0.0f;
    float y = 0.0f;
    float angle = 0.0f;

    Pose();
    Pose(float x, float y, float angle);
};

struct SensorScans
{
    float left = 0.0f;
    float right = 0.0f;
    float front = 0.0f;
    float back = 0.0f;
    float angle = 0.0f;
};
#endif