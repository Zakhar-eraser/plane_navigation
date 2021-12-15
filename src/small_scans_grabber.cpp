#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PointStamped.h>
#include <plane_na>

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

void RangeCallback(sensor_msgs::RangeConstPtr msg)
{
    if(msg->header.frame_id == "range_front")
    {
        scans->front = msg->range;
        frontUpdated = true;
    }else if(msg->header.frame_id == "range_left")
    {
        scans->left = msg->range;
        leftUpdated = true;
    }else if(msg->header.frame_id == "range_right")
    {
        scans->right = msg->range;
        rightUpdated = true;
    }else
    {
        scans->back = msg->range;
    }
}

void AngleCallback(std_msgs::Float32ConstPtr msg)
{
    scans->angle = msg->data;
    angleUpdated = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_grabber");
    n = new ros::NodeHandle();
    scans = new SensorScans();

    geometry_msgs::PointStamped data;

    dataPub = n->advertise<geometry_msgs::PointStamped>("estimated_pose", 1);

    frontSub = n->subscribe<sensor_msgs::Range>("/range_front", 1, RangeCallback);
    leftSub = n->subscribe<sensor_msgs::Range>("/range_left", 1, RangeCallback);
    rightSub = n->subscribe<sensor_msgs::Range>("/range_right", 1, RangeCallback);
    angleSub = n->subscribe<std_msgs::Float32>("/angle", 1, AngleCallback);

    ros::Rate rate(30);



    while(ros::ok())
    {
        if(frontUpdated && leftUpdated && rightUpdated && angleUpdated)
        {
            data.header.stamp = ros::Time::now();
            frontUpdated = leftUpdated = rightUpdated = angleUpdated = false;
            dataPub.publish(data);
        }
    }
    
    delete scans;
    delete n;
    return 0;
}