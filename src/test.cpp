#include "plane_navigation.hpp"
#include <iostream>

int main()
{
    std::string data;
    Pose initPose;
    SensorScans *scans = new SensorScans();
    Navigator nav("/home/zakhar/catkin_ws/src/plane_navigation/config/map.yaml", scans);
    do
    {
        std::cout << "Initial position:\n";
        std::cin >> initPose.x >> initPose.y;
        std::cout << "\nRelative angle: \n";
        std::cin >> scans->angle;
        std::cout << "\nLaser Ranges (L F R B):\n";
        std::cin >> scans->left >> scans->front >> scans->right >> scans->back;
        nav.CalculatePose();
        initPose = nav.GetMinDiversePosition(initPose);
        std::cout << "\nPosition: " << initPose.x << " " << initPose.y << std::endl;
        std::cout << "Orientation: " << initPose.angle << std::endl;
        std::cout << "Continue? (yes/no)   ";
        std::cin >> data;
    } while (data != "no");
    
    delete scans;
    return 0;
}