#include <iostream>
#include <fstream>
#include "RangefinderManager/RangefinderManager.hpp"
#include <vector>
#include <yaml-cpp/yaml.h>

using pair = std::pair<float, float>;

int main(int argc, char **argv)
{
    std::string path = "../../plane_navigation/config/";
    if(argc > 1)
    {
        path += argv[1];
        path += ".yaml";
    }else path += "map.yaml";
    std::ofstream mapFile(path);

    YAML::Node lasersConfig = YAML::LoadFile("../../plane_navigation/config/robot_config.yaml");
    pair leftOs = lasersConfig["left"]["offsets"].as<pair>();
    pair rightOs = lasersConfig["right"]["offsets"].as<pair>();
    pair frontOs = lasersConfig["front"]["offsets"].as<pair>();
    pair backOs = lasersConfig["back"]["offsets"].as<pair>();

    RangefinderManager* rfManager = RangefinderManager::GetInstance();
    rfManager->addSensorAddress(0x60, SensorTypes::GARMIN_LIDAR); // left
	rfManager->addSensorAddress(0x62, SensorTypes::GARMIN_LIDAR); // right
	rfManager->addSensorAddress(0x10, SensorTypes::BENEWAKE_LIDAR); // back
	rfManager->addSensorAddress(0x12, SensorTypes::BENEWAKE_LIDAR); // front
	rfManager->startRecive();

    std::vector<float> rangesArray;
    float left, right, front, back;
    float x, y;

    char key = 'n';

    while(key != 'y')
    {
        rangesArray = rfManager->getDistanceList();
        left = rangesArray[0];
        right = rangesArray[1];
        front = rangesArray[3];
        back = rangesArray[2];
        x = (left + right - leftOs.first + rightOs.first);
        y = (front + back - backOs.second + frontOs.second);
        std::cout << "Width: " << x << "\nLength: " << y << std::endl;
        std::cout << "Create map? [y/n]" << std::endl;
        std::cin >> key;
    }
    x /= 2;
    y /= 2;
    YAML::Emitter emitter(mapFile);
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "lines" << YAML::Value;
    emitter << YAML::BeginMap;
    //Line1 description
    emitter << YAML::Key << "line1" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "start" << YAML::Value << YAML::Flow << YAML::BeginSeq << -x << -y << YAML::EndSeq;
    emitter << YAML::Key << "end" << YAML::Value << YAML::Flow << YAML::BeginSeq << -x << y << YAML::EndSeq;
    emitter << YAML::Key << "angle" << YAML::Value << 0 << YAML::EndMap;
    //Line2 description
    emitter << YAML::Key << "line2" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "start" << YAML::Value << YAML::Flow << YAML::BeginSeq << -x << y << YAML::EndSeq;
    emitter << YAML::Key << "end" << YAML::Value << YAML::Flow << YAML::BeginSeq << x << y << YAML::EndSeq;
    emitter << YAML::Key << "angle" << YAML::Value << 4.71239f << YAML::EndMap;
    //Line3 description
    emitter << YAML::Key << "line3" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "start" << YAML::Value << YAML::Flow << YAML::BeginSeq << x << y << YAML::EndSeq;
    emitter << YAML::Key << "end" << YAML::Value << YAML::Flow << YAML::BeginSeq << x << -y << YAML::EndSeq;
    emitter << YAML::Key << "angle" << YAML::Value << 3.14159f << YAML::EndMap;
    //Line4 description
    emitter << YAML::Key << "line4" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "start" << YAML::Value << YAML::Flow << YAML::BeginSeq << x << -y << YAML::EndSeq;
    emitter << YAML::Key << "end" << YAML::Value << YAML::Flow << YAML::BeginSeq << -x << -y << YAML::EndSeq;
    emitter << YAML::Key << "angle" << YAML::Value << 1.5708f << YAML::EndMap;
    emitter << YAML::EndMap;
    emitter << YAML::Key << "initial_pose" << YAML::Value;
    emitter << YAML::Flow << YAML::BeginSeq << -x + left + leftOs.first <<
                                               -y + front + frontOs.second << YAML::EndSeq;
    emitter << YAML::EndMap;
    mapFile.close();
    return 0;
}