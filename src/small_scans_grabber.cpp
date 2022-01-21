#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <plane_navigation.hpp>
#include <yaml-cpp/yaml.h>

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

Navigator *nav;

ros::Publisher dataPub;

geometry_msgs::PoseStamped data;

YAML::Node frameIds;

SensorScans *scans;

void RangeCallback(sensor_msgs::RangeConstPtr msg)
{
    if(msg->header.frame_id == frameIds["front_id"].as<std::string>())
    {
        scans->frontLaser.range = msg->range;
        frontUpdated = true;
    }else if(msg->header.frame_id == frameIds["left_id"].as<std::string>())
    {
        scans->leftLaser.range = msg->range;
        leftUpdated = true;
    }else if(msg->header.frame_id == frameIds["right_id"].as<std::string>())
    {
        scans->rightLaser.range = msg->range;
        rightUpdated = true;
    }else
    {
        scans->backLaser.range = msg->range;
        backUpdated = true;
    }
}

void AngleCallback(std_msgs::Float32ConstPtr msg)
{
    scans->yaw = msg->data;
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
    //Read what lasers frame ids to expect
    frameIds = YAML::LoadFile("../../plane_navigation/config/sensors_frame_id.yaml");
    //Read initial pose of drone
    YAML::Node initPose = YAML::LoadFile("../../plane_navigation/config/initial_pose.yaml");

    dataPub = n->advertise<geometry_msgs::PoseStamped>(topicsNode["pose_topic"].as<std::string>(), 1);

    frontSub = n->subscribe<sensor_msgs::Range>(topicsNode["range_front_topic"].as<std::string>(), 1, RangeCallback);
    leftSub = n->subscribe<sensor_msgs::Range>(topicsNode["range_left_topic"].as<std::string>(), 1, RangeCallback);
    rightSub = n->subscribe<sensor_msgs::Range>(topicsNode["range_right_topic"].as<std::string>(), 1, RangeCallback);
    backSub = n->subscribe<sensor_msgs::Range>(topicsNode["range_back_topic"].as<std::string>(), 1, RangeCallback);
    angleSub = n->subscribe<std_msgs::Float32>(topicsNode["yaw_topic"].as<std::string>(), 1, AngleCallback);


    ros::Rate rate(20);

    Pose lastPose(initPose["x"].as<float>(), initPose["y"].as<float>(), initPose["yaw"].as<float>());

    nav = new Navigator("../../plane_navigation/config/map.yaml", scans);

    // nav.StartNavigator();

    while(ros::ok())
    {
        if((frontUpdated || !scans->frontLaser.isOn) && (leftUpdated || !scans->leftLaser.isOn) &&
           (rightUpdated || !scans->rightLaser.isOn) &&  (backUpdated || !scans->backLaser.isOn) && angleUpdated)
        {
            nav->isUpdate = true;
            nav->CalculatePoses();
            data.header.stamp = ros::Time::now();
            lastPose = nav->GetMinDiversePosition(lastPose);
            data.pose.position.x = lastPose.position.x;
            data.pose.position.y = lastPose.position.y;
            data.pose.orientation.x = lastPose.angle;
            dataPub.publish(data);
            angleUpdated = backUpdated = frontUpdated = leftUpdated = rightUpdated = false;
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