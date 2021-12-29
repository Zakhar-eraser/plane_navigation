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

ros::Publisher dataPub;

YAML::Node frameIds;

SensorScans *scans;
SensorScans temp;

void RangeCallback(sensor_msgs::RangeConstPtr msg)
{
    if(msg->header.frame_id == frameIds["front_id"].as<std::string>())
    {
        temp.front = msg->range;
        frontUpdated = true;
    }else if(msg->header.frame_id == frameIds["left_id"].as<std::string>())
    {
        temp.left = msg->range;
        leftUpdated = true;
    }else if(msg->header.frame_id == frameIds["right_id"].as<std::string>())
    {
        temp.right = msg->range;
        rightUpdated = true;
    }else
    {
        temp.back = msg->range;
        backUpdated = true;
    }
}

void AngleCallback(std_msgs::Float32ConstPtr msg)
{
    temp.yaw = msg->data;
    angleUpdated = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_grabber");
    n = new ros::NodeHandle();
    scans = new SensorScans();

    geometry_msgs::PoseStamped data;
    //Node with topic paths
    YAML::Node topicsNode = YAML::LoadFile("../../plane_navigation/config/topics.yaml");
    //What sensors must be accounted. Read from a yaml file and write the data to a Switcher structur
    YAML::Node switcherNode = YAML::LoadFile("../../plane_navigation/config/sensor_switcher.yaml");
    Switcher switcher(switcherNode["use_left"].as<bool>(), switcherNode["use_right"].as<bool>(), switcherNode["use_back"].as<bool>());
    //Sensors offsets. Read from a yaml file and write the data to a Offsets structure
    YAML::Node sensorTfNode = YAML::LoadFile("../../plane_navigation/config/sensors_tf.yaml");
    Offsets sensorsTf(sensorTfNode["front"].as<pair>(), sensorTfNode["back"].as<pair>(), sensorTfNode["left"].as<pair>(), sensorTfNode["right"].as<pair>());
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


    ros::Rate rate(10);

    Pose lastPose(initPose["x"].as<float>(), initPose["y"].as<float>(), 0);

    Navigator nav("../../plane_navigation/config/map.yaml", scans, switcher, sensorsTf);

    // nav.StartNavigator();

    while(ros::ok())
    {
        if(frontUpdated && (leftUpdated || !switcher.left) && (rightUpdated || !switcher.right) &&  (backUpdated || !switcher.back) && angleUpdated)
        {
            nav.isUpdate = true;
            scans->back = temp.back;
            scans->front = temp.front;
            scans->left = temp.left;
            scans->right = temp.right;
            scans->yaw = temp.yaw;
            scans->pitch = scans->pitch;
            scans->roll = scans->roll;
            nav.CalculatePoses();
            data.header.stamp = ros::Time::now();
            lastPose = nav.GetMinDiversePosition(lastPose);
            data.pose.position.x = lastPose.x;
            data.pose.position.y = lastPose.y;
            data.pose.orientation.x = lastPose.angle;
            dataPub.publish(data);
            angleUpdated = backUpdated = frontUpdated = leftUpdated = rightUpdated = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    delete scans;
    delete n;
    return 0;
}