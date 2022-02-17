#include "plane_navigation.hpp"
#include <iostream>
#include <yaml-cpp/yaml.h>

int main()
{
    SensorScans *scans = new SensorScans();
    Navigator nav("../../plane_navigation/config/test_map.yaml", scans);
    YAML::Node params = YAML::LoadFile("../../plane_navigation/config/test_config.yaml");
    YAML::Node leftParams = params["left"];
    YAML::Node rightParams = params["right"];
    YAML::Node frontParams = params["front"];
    YAML::Node backParams = params["back"];

    scans->leftLaser.isOn = leftParams["is_on"].as<bool>();
    scans->leftLaser.offsets = leftParams["offsets"].as<std::pair<float, float>>();
    scans->leftLaser.range = leftParams["range"].as<float>();

    scans->rightLaser.isOn = rightParams["is_on"].as<bool>();
    scans->rightLaser.offsets = rightParams["offsets"].as<std::pair<float, float>>();
    scans->rightLaser.range = rightParams["range"].as<float>();

    scans->frontLaser.isOn = frontParams["is_on"].as<bool>();
    scans->frontLaser.offsets = frontParams["offsets"].as<std::pair<float, float>>();
    scans->frontLaser.range = frontParams["range"].as<float>();

    scans->backLaser.isOn = backParams["is_on"].as<bool>();
    scans->backLaser.offsets = backParams["offsets"].as<std::pair<float, float>>();
    scans->backLaser.range = backParams["range"].as<float>();

    scans->yaw = params["yaw"].as<float>();

    nav.CalculatePose();
    Pose pose = nav.GetPose();
    std::cout << "X: " << pose.position.x << std::endl;
    std::cout << "Y: " << pose.position.y << std::endl;
    std::cout << "Yaw: " << pose.angle << std::endl;

    delete scans;
    return 0;
}