#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <plane_navigation.hpp>

ros::NodeHandle *n;

ros::Subscriber frontSub;
ros::Subscriber leftSub;
ros::Subscriber rightSub;
ros::Subscriber angleSub;

bool frontUpdated = false;
bool leftUpdated = false;
bool rightUpdated = false;
bool angleUpdated = false;

ros::Publisher dataPub;

SensorScans *scans;
SensorScans temp;

void RangeCallback(sensor_msgs::RangeConstPtr msg)
{
    if(msg->header.frame_id == "range_front_right")
    {
        temp.front = msg->range;
        frontUpdated = true;
    }else if(msg->header.frame_id == "range_left")
    {
        temp.left = msg->range;
        leftUpdated = true;
    }else if(msg->header.frame_id == "range_right")
    {
        temp.right = msg->range;
        rightUpdated = true;
    }else
    {
        temp.back = msg->range;
    }
}

void AngleCallback(std_msgs::Float32ConstPtr msg)
{
    scans->angle = 0; //msg->data;
    angleUpdated = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_grabber");
    n = new ros::NodeHandle();
    scans = new SensorScans();

    geometry_msgs::PoseStamped data;

    dataPub = n->advertise<geometry_msgs::PoseStamped>("estimated_pose", 1);

    frontSub = n->subscribe<sensor_msgs::Range>("/range_front_right", 1, RangeCallback);
    leftSub = n->subscribe<sensor_msgs::Range>("/range_left", 1, RangeCallback);
    rightSub = n->subscribe<sensor_msgs::Range>("/range_right", 1, RangeCallback);
    angleSub = n->subscribe<std_msgs::Float32>("/angle", 1, AngleCallback);

    ros::Rate rate(30);

    std::vector<float> startPos(2);
    n->getParam("start_position", startPos);
    Pose lastPose(0, 0, 0);

    Navigator nav("/home/argus/catkin_ws/src/ScanController/plane_navigation/config/map.yaml", scans);
    nav.StartNavigator();

    while(ros::ok())
    {
        scans->angle = 0; //msg->data;
        angleUpdated = true;
        if(frontUpdated && leftUpdated && rightUpdated && angleUpdated)
        {
            *scans = temp;
            data.header.stamp = ros::Time::now();
            lastPose = nav.GetMinDiversePosition(lastPose);
            data.pose.position.x = lastPose.x;
            data.pose.position.y = lastPose.y;
            data.pose.orientation.x = lastPose.angle;
            dataPub.publish(data);
            frontUpdated = leftUpdated = rightUpdated = angleUpdated = false;
        }
        ros::spinOnce();
    }
    
    delete scans;
    delete n;
    return 0;
}