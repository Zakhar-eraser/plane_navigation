#include <ros/ros.h>
#include "PointCloud2Access.hpp"
#include "rsOrientation.hpp"
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseStamped.h>
#include <rs_orientation/DroneSensors.h>

std::map<std::string, Segment> map;

std::string angleTopic;
std::string distanceTopic;
std::string poseTopic;
std::string mapConfig;

std::pair<float, float> prevPose;

ros::NodeHandle *n;
ros::Subscriber sensorsSub;
ros::Publisher posePub;

YAML::Node node;

void Callback(rs_orientation::DroneSensorsConstPtr msg)
{
    std::pair<float, float> = GetPosition(map, )
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs_orientation_node");
    n = new ros::NodeHandle();
    sensorsSub = n->subscribe<rs_orientation::DroneSensors>(angleTopic, 1, Callback);
    posePub = n->advertise<geometry_msgs::PoseStamped>(poseTopic, 1);
    n->getParam("map_config_file", mapConfig);
    n->
    std::map<std::string, Segment> map;
    node = YAML::LoadFile(mapConfig)["lines"];
    for(YAML::const_iterator i = node.begin(); i != node.end(); ++i)
    {
        std::string key = i->first.as<std::string>();
        std::pair<float, float> start = i->second["start"].as<std::pair<float, float>>();
        std::pair<float, float> end = i->second["end"].as<std::pair<float, float>>();
        float angle = i->second["angle"].as<float>();
        map[key] = Segment(start, end, angle);
    }
    delete n;
    return 0;
}