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

bool calibrated;

Navigator *nav;

ros::Publisher dataPub;

geometry_msgs::PoseStamped data;

YAML::Node rosConfig;

SensorScans *scans;

void RangeCallback(sensor_msgs::RangeConstPtr msg)
{
    if(msg->header.frame_id == rosConfig["front_id"].as<std::string>())
    {
        scans->frontLaser.range = msg->range;
        frontUpdated = true;
    }else if(msg->header.frame_id == rosConfig["left_id"].as<std::string>())
    {
        scans->leftLaser.range = msg->range;
        leftUpdated = true;
    }else if(msg->header.frame_id == rosConfig["right_id"].as<std::string>())
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

    //What sensors must be accounted. Read from a yaml file and write the data to a Switcher structur
    YAML::Node laserConfig = YAML::LoadFile("../../plane_navigation/config/robot_config.yaml");
    scans->leftLaser.isOn = laserConfig["left"]["is_on"].as<bool>();
    scans->rightLaser.isOn = laserConfig["right"]["is_on"].as<bool>();
    scans->backLaser.isOn = laserConfig["back"]["is_on"].as<bool>();
    scans->frontLaser.isOn = laserConfig["front"]["is_on"].as<bool>();
    //Sensors offsets. Read from a yaml file and write the data to a Offsets structure
    scans->leftLaser.offsets = laserConfig["left"]["offsets"].as<pair>();
    scans->rightLaser.offsets = laserConfig["right"]["offsets"].as<pair>();
    scans->backLaser.offsets = laserConfig["back"]["offsets"].as<pair>();
    scans->frontLaser.offsets = laserConfig["front"]["offsets"].as<pair>();
    //Read what lasers frame ids to expect
    rosConfig = YAML::LoadFile("../../plane_navigation/config/ros_config.yaml");

    dataPub = n->advertise<geometry_msgs::PoseStamped>(rosConfig["pose_topic"].as<std::string>(), 1);

    frontSub = n->subscribe<sensor_msgs::Range>(rosConfig["front_topic"].as<std::string>(), 1, RangeCallback);
    leftSub = n->subscribe<sensor_msgs::Range>(rosConfig["left_topic"].as<std::string>(), 1, RangeCallback);
    rightSub = n->subscribe<sensor_msgs::Range>(rosConfig["right_topic"].as<std::string>(), 1, RangeCallback);
    backSub = n->subscribe<sensor_msgs::Range>(rosConfig["back_topic"].as<std::string>(), 1, RangeCallback);
    angleSub = n->subscribe<std_msgs::Float32>(rosConfig["yaw_topic"].as<std::string>(), 1, AngleCallback);

    ros::Rate rate(20);

    nav = new Navigator("../../plane_navigation/config/map.yaml", scans);
    // nav.StartNavigator();
    Pose lastPose;
    while(ros::ok())
    {
        if((frontUpdated || !scans->frontLaser.isOn) && (leftUpdated || !scans->leftLaser.isOn) &&
           (rightUpdated || !scans->rightLaser.isOn) &&  (backUpdated || !scans->backLaser.isOn) && angleUpdated)
        {
            //nav->isUpdate = true;
            if(!calibrated)
            {
                nav->CalibrateWidthLength(M_PI_2);
                calibrated = true;
            }
            nav->CalculatePoseByRangefinders();
            lastPose = nav->GetPose();
            data.header.stamp = ros::Time::now();
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