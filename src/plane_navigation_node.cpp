#include <ros/ros.h>
#include "plane_navigation.hpp"
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseStamped.h>
#include <plane_navigation/DroneSensors.h>

std::map<std::string, Segment> map;
plane_navigation::DroneSensors scans;

std::string sensorsDataTopic;
std::string poseTopic;
std::string mapConfig;

std::pair<float, float> prevPose;
geometry_msgs::PoseStamped pose;

ros::NodeHandle *n;
ros::Subscriber sensorsSub;
ros::Publisher posePub;

YAML::Node node;

void Callback(plane_navigation::DroneSensorsConstPtr msg)
{
    prevPose = GetPosition(map, prevPose, msg);
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = prevPose.first;
    pose.pose.position.y = prevPose.second;
    posePub.publish(pose);
}

int main(int argc, char **argv)
{
    std::vector<float> coords;
    ros::init(argc, argv, "plane_navigation_node");
    n = new ros::NodeHandle();
    n->getParam("map_config_file", mapConfig);
    n->getParam("initial_position", coords);
    n->getParam("sensors_data_topic", sensorsDataTopic);
    n->getParam("pose_topic", poseTopic);
    sensorsSub = n->subscribe<plane_navigation::DroneSensors>(sensorsDataTopic, 1, Callback);
    posePub = n->advertise<geometry_msgs::PoseStamped>(poseTopic, 1);
    prevPose.first = coords[0];
    prevPose.second = coords[1];
    
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
    
    ros::spin();
    delete n;
    return 0;
}