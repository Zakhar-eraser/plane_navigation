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

ros::NodeHandle *n;
ros::Subscriber sensorsSub;
ros::Publisher posePub;

YAML::Node node;

void Callback(rs_orientation::DroneSensors msg)
{
   // GetPosition();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs_orientation_node");
    n = new ros::NodeHandle();
    sensorsSub = n->subscribe<rs_orientation::DroneSensors>(angleTopic, 1, Callback);
    posePub = n->advertise<geometry_msgs::PoseStamped>(poseTopic, 1);
    n->getParam("map_config_file", mapConfig);
    //node = YAML::LoadFile(mapConfig);
    //std::string key;
    //for(YAML::const_iterator i = node["lines"].begin(); i != node["lines"].end(); ++i)
    //{
    //    key = i->first.as<std::string>();
    //    Segment wall(std::make_pair<float, float>(i->second["start"][0], i->second["start"][1]))
    //}
    delete n;
    return 0;
}