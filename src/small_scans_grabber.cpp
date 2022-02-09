#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <plane_navigation.hpp>
#include <yaml-cpp/yaml.h>
#include "RangefinderManager/RangefinderManager.hpp"

ros::NodeHandle *n;

ros::Subscriber frontSub;
ros::Subscriber leftSub;
ros::Subscriber rightSub;
ros::Subscriber backSub;
ros::Subscriber angleSub;

bool frontUpdated = false;
bool leftUpdated = false;
bool rightUpdated = false;
bool backUpdated = false;
bool angleUpdated = false;

bool calibrated = true;

Navigator *nav;

ros::Publisher dataPub;

geometry_msgs::PoseStamped data;

YAML::Node frameIds;

SensorScans *scans;

//void RangeCallback(sensor_msgs::RangeConstPtr msg)
//{
//    if(msg->header.frame_id == frameIds["front_id"].as<std::string>())
//    {
//        scans->front = msg->range;
//        frontUpdated = true;
//    }else if(msg->header.frame_id == frameIds["left_id"].as<std::string>())
//    {
//        scans->left = msg->range;
//        leftUpdated = true;
//    }else if(msg->header.frame_id == frameIds["right_id"].as<std::string>())
//    {
//        scans->right = msg->range;
//        rightUpdated = true;
//    }else
//    {
//        scans->back = msg->range;
//        backUpdated = true;
//    }
//}

void AngleCallback(nav_msgs::OdometryConstPtr msg)
{
    data.pose.position.z = msg->pose.pose.position.z;
    float w = msg->pose.pose.orientation.w;
    float x = msg->pose.pose.orientation.x;
    float y = msg->pose.pose.orientation.y;
    float z = msg->pose.pose.orientation.z;

    data.pose.orientation.w = w;
    data.pose.orientation.x = x;
    data.pose.orientation.y = y;
    data.pose.orientation.z = z;
    scans->yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    angleUpdated = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_grabber");
    n = new ros::NodeHandle();
    scans = new SensorScans();

    //Node with topic paths
    YAML::Node topicsNode = YAML::LoadFile("../../plane_navigation/config/topics.yaml");
    //What sensors must be accounted. Read from a yaml file and write the data to a Switcher structur
    YAML::Node switcherNode = YAML::LoadFile("../../plane_navigation/config/sensor_switcher.yaml");
    scans->leftLaser.isOn = switcherNode["use_left"].as<bool>();
    scans->rightLaser.isOn = switcherNode["use_right"].as<bool>();
    scans->backLaser.isOn = switcherNode["use_back"].as<bool>();
    scans->frontLaser.isOn = switcherNode["use_front"].as<bool>();
    //Sensors offsets. Read from a yaml file and write the data to a Offsets structure
    YAML::Node sensorTfNode = YAML::LoadFile("../../plane_navigation/config/sensors_tf.yaml");
    scans->leftLaser.offsets = sensorTfNode["left"].as<pair>();
    scans->rightLaser.offsets = sensorTfNode["right"].as<pair>();
    scans->backLaser.offsets = sensorTfNode["back"].as<pair>();
    scans->frontLaser.offsets = sensorTfNode["front"].as<pair>();
    //Read initial pose of drone
    YAML::Node initPose = YAML::LoadFile("../../plane_navigation/config/initial_pose.yaml");

    dataPub = n->advertise<geometry_msgs::PoseStamped>(topicsNode["pose_topic"].as<std::string>(), 1);

    RangefinderManager* rfManager = RangefinderManager::GetInstance();
	
	rfManager->addSensorAddress(0x60, SensorTypes::GARMIN_LIDAR); // left
	rfManager->addSensorAddress(0x62, SensorTypes::GARMIN_LIDAR); // right
	rfManager->addSensorAddress(0x10, SensorTypes::BENEWAKE_LIDAR); // back
	rfManager->addSensorAddress(0x12, SensorTypes::BENEWAKE_LIDAR); // front
	rfManager->startRecive();
    // rfManager->addSensorAddress(0x14);
    //frontSub = n->subscribe<sensor_msgs::Range>(topicsNode["range_front_topic"].as<std::string>(), 1, RangeCallback);
    //leftSub = n->subscribe<sensor_msgs::Range>(topicsNode["range_left_topic"].as<std::string>(), 1, RangeCallback);
    //rightSub = n->subscribe<sensor_msgs::Range>(topicsNode["range_right_topic"].as<std::string>(), 1, RangeCallback);
    //backSub = n->subscribe<sensor_msgs::Range>(topicsNode["range_back_topic"].as<std::string>(), 1, RangeCallback);
    angleSub = n->subscribe<nav_msgs::Odometry>(topicsNode["yaw_topic"].as<std::string>(), 1, AngleCallback);

    ros::Rate rate(20);

    std::vector<float> rangesList;

    nav = new Navigator("../../plane_navigation/config/map.yaml", scans);
    nav->SetLastPose(Pose(initPose["x"].as<float>(), initPose["y"].as<float>(), initPose["yaw"].as<float>()));
    // nav.StartNavigator();
    Pose lastPose;
    while(ros::ok())
    {
        if(angleUpdated)
        {
            rangesList = rfManager->getDistanceList();
            scans->leftLaser.range = rangesList[0];
            scans->frontLaser.range = rangesList[3];
            scans->rightLaser.range = rangesList[1];
            nav->isUpdate = true;
            if(!calibrated) {nav->CalibrateSymmetricMap(); calibrated = true;}
            nav->CalculatePose();
            lastPose = nav->GetPose();
            data.header.stamp = ros::Time::now();
            data.pose.position.x = lastPose.position.x;
            data.pose.position.y = lastPose.position.y;
            data.pose.orientation.x = lastPose.angle;
            dataPub.publish(data);
            angleUpdated = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
    delete scans;
    delete nav;
    delete n;
    return 0;
}