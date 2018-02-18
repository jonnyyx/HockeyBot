#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <math.h> //fuer cos und sin
#define PI 3.14159265

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sendCalibration"); // Initiate new ROS node named "sendCalibration"
    ros::NodeHandle node;

    ros::Publisher corner1_pub = node.advertise<geometry_msgs::Twist>("corner1_topic", 1);
    ros::Publisher corner2_pub = node.advertise<geometry_msgs::Twist>("corner2_topic", 1);
    ros::Publisher corner3_pub = node.advertise<geometry_msgs::Twist>("corner3_topic", 1);
    ros::Publisher corner4_pub = node.advertise<geometry_msgs::Twist>("corner4_topic", 1);

    geometry_msgs::Twist p1,p2,p3,p4;
    p1.linear.x = -0.5;
    p1.linear.y = 1;

    p2.linear.x = 2.5;
    p2.linear.y = 1;

    p3.linear.x = 2.5;
    p3.linear.y = -1;

    p4.linear.x = -0.5;
    p4.linear.y = -1;

    double theta = (30.0/180)*PI;
    p1.angular.z = theta;
    p2.angular.z = theta;
    p3.angular.z = theta;
    p4.angular.z = theta;


    ros::Rate loop_rate(0.1);
    loop_rate.sleep();
    int count = 0;
    while (ros::ok()&& count<2) // Publish 2 times
    {
        ROS_INFO("corner1 publish");
        corner1_pub.publish(p1);
        ros::spinOnce();

        ROS_INFO("corner2 publish");
        corner2_pub.publish(p2);
        ros::spinOnce();

        ROS_INFO("corner3 publish");
        corner3_pub.publish(p3);
        ros::spinOnce();

        ROS_INFO("corner4 publish");
        corner4_pub.publish(p4);
        ros::spinOnce();

        loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
        count++;
    }

    return 0;
}

