#include "plane_navigation.hpp"
#include <yaml-cpp/yaml.h>
#include "plane_navigation/DroneSensors.h"

int main()
{
    std::map<std::string, Segment> map;
    YAML::Node node = YAML::LoadFile("/home/{user_name}/catkin_ws/src/plane_navigation/config/map.yaml")["lines"];
    for(YAML::const_iterator i = node.begin(); i != node.end(); ++i)
    {
        std::string key = i->first.as<std::string>();
        std::pair<float, float> start = i->second["start"].as<std::pair<float, float>>();
        std::pair<float, float> end = i->second["end"].as<std::pair<float, float>>();
        float angle = i->second["angle"].as<float>();
        map[key] = Segment(start, end, angle);
    }
    std::string data;
    std::pair<float, float> initPos;
    float left, front, right, back, angle;
    plane_navigation::DroneSensorsPtr msg(new plane_navigation::DroneSensors());
    do
    {
        std::cout << "Initial position:\n";
        std::cin >> initPos.first >> initPos.second;
        std::cout << "\nLaser Ranges (L F R B):\n";
        std::cin >> left >> front >> right >> back;
        std::cout << "\nRelative angle: \n";
        std::cin >> angle;
        msg->left.range = left;
        msg->right.range = right;
        msg->front.range = front;
        msg->back.range = back;
        msg->angle.data = angle;
        initPos = GetPosition(map, initPos, (plane_navigation::DroneSensorsConstPtr)msg);
        std::cout << "\nPosition: " << initPos.first << " " << initPos.second << std::endl;
        std::cout << "Continue? (yes/no)   ";
        std::cin >> data;
    } while (data != "no");
    

    return 0;
}