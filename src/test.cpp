#include "rsOrientation.hpp"
#include <yaml-cpp/yaml.h>
#include "rs_orientation/DroneSensors.h"

int main()
{
    std::map<std::string, Segment> map;
    YAML::Node node = YAML::LoadFile("/home/zakhar/catkin_ws/src/rs_orientation/config/map.yaml")["lines"];
    //for(const auto &line : node)
    //{
    //    std::string key = line.first.as<std::string>();
    //    std::pair<float, float> start = line.second["start"].as<std::pair<float, float>>();
    //    std::pair<float, float> end = line.second["end"].as<std::pair<float, float>>();
    //    float angle = line.second["angle"].as<float>();
    //    map[key] = Segment(start, end, angle);
    //}
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
    rs_orientation::DroneSensorsPtr msg(new rs_orientation::DroneSensors());
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
        initPos = GetPosition(map, initPos, (rs_orientation::DroneSensorsConstPtr)msg);
        std::cout << "\nPosition: " << initPos.first << " " << initPos.second << std::endl;
        std::cout << "Continue? (yes/no)   ";
        std::cin >> data;
    } while (data != "no");
    

    return 0;
}