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

    YAML::Node offsets = YAML::LoadFile("../../plane_navigation/config/sensors_tf.yaml");
    pair leftOs = offsets["left"].as<pair>();
    pair rightOs = offsets["right"].as<pair>();
    pair frontOs = offsets["front"].as<pair>();
    pair backOs = offsets["back"].as<pair>();

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
    mapFile << "lines:\n";
    //Line1 description
    mapFile << "  line1: \n";
    mapFile << "    start: [" << -x << ", " << -y << "]\n";
    mapFile << "    end: [" << -x << ", " << y << "]\n";
    mapFile << "    angle: " << 0 << std::endl;
    //Line2 description
    mapFile << "  line2: \n";
    mapFile << "    start: [" << -x << ", " << y << "]\n";
    mapFile << "    end: [" << x << ", " << y << "]\n";
    mapFile << "    angle: " << 4.7123889804 << std::endl;
    //Line3 description
    mapFile << "  line3: \n";
    mapFile << "    start: [" << x << ", " << y << "]\n";
    mapFile << "    end: [" << x << ", " << -y << "]\n";
    mapFile << "    angle: " << 3.14159265359 << std::endl;
    //Line4 description
    mapFile << "  line4: \n";
    mapFile << "    start: [" << x << ", " << -y << "]\n";
    mapFile << "    end: [" << -x << ", " << -y << "]\n";
    mapFile << "    angle: " << 1.57079632679 << std::endl;
    mapFile.close();
    return 0;
}