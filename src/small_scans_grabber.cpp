#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <plane_navigation/DroneSensors.h>

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

plane_navigation::DroneSensors data;

void RangeCallback(sensor_msgs::RangeConstPtr msg)
{
    data.header.stamp = ros::Time::now();
    if(msg->header.frame_id == "range_front")
    {
        data.front = *msg;
        frontUpdated = true;
    }else if(msg->header.frame_id == "range_left")
    {
        data.left = *msg;
        leftUpdated = true;
    }else if(msg->header.frame_id == "range_right")
    {
        data.right = *msg;
        rightUpdated = true;
    }else
    {
        data.back = *msg;
    }
}

void AngleCallback(std_msgs::Float32ConstPtr msg)
{
    data.header.stamp = ros::Time::now();
    data.angle = *msg;
    angleUpdated = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_grabber");
    n = new ros::NodeHandle();

    dataPub = n->advertise<plane_navigation::DroneSensors>("/sensors_data", 1);

    frontSub = n->subscribe<sensor_msgs::Range>("/range_front", 1, RangeCallback);
    leftSub = n->subscribe<sensor_msgs::Range>("/range_left", 1, RangeCallback);
    rightSub = n->subscribe<sensor_msgs::Range>("/range_right", 1, RangeCallback);
    angleSub = n->subscribe<std_msgs::Float32>("/angle", 1, AngleCallback);

    ros::Rate rate(30);

    while(ros::ok())
    {
        if(frontUpdated && leftUpdated && rightUpdated && angleUpdated)
        {
            frontUpdated = leftUpdated = rightUpdated = angleUpdated = false;
            dataPub.publish(data);
        }
    }
    
    delete n;
    return 0;
}