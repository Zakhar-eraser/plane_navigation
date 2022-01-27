#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <iostream>

bool leftUpdated, rightUpdated;
float leftRange, rightRange;

void RecieveRange(sensor_msgs::RangeConstPtr msg)
{
    if(msg->header.frame_id == "range_front_left")
    {
        leftUpdated = true;
        leftRange = msg->range;
    }
    else
    {
        rightUpdated = true;
        rightRange = msg->range;
    }
}

int main(int c, char **v)
{
    ros::init(c, v, "yaw");
    ros::NodeHandle n;

    ros::Subscriber leftRangeSub = n.subscribe<sensor_msgs::Range>("/range_front_left", 1, RecieveRange);
    ros::Subscriber rightRangeSub = n.subscribe<sensor_msgs::Range>("/range_front_right", 1, RecieveRange);

    ros::Publisher yawPub = n.advertise<std_msgs::Float32>("/yaw", 1);

    ros::Rate rate(20);

    std_msgs::Float32 yaw;
    while(ros::ok())
    {
        if(rightUpdated && leftUpdated)
        {
            rightUpdated = leftUpdated = false;
            yaw.data = atan2(leftRange - rightRange, 0.4);
            yawPub.publish(yaw);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}