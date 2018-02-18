#ifndef CARTOGRAPHER_H
#define CARTOGRAPHER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
#include "std_msgs/UInt8.h"
#include "math.h"
#include "geometry_msgs/Twist.h"

#define NB_CONES_AB 3
#define MIN_WIDTH_CONES 2
#define DEF_RANGE_CONE 5.0

#define LEFT_OFFSET_ARRAY 0
#define RIGHT_OFFSET_ARRAY 3

#define abs(x) ((x)<0 ? -(x) : (x))

class Cartographer{

public:
    Cartographer();
    ~Cartographer();
    void start();

    bool getIsInitialized();
    float getFieldRatio();

private: 
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void scanSideForCones(const sensor_msgs::LaserScan::ConstPtr &scan, double minRad, double maxRad, cv::Point2f* cones_coordinates_array, int array_offset);
    void delimitField(const sensor_msgs::LaserScan_<std::allocator<void> >::ConstPtr &scan);

private:
    /* Node */
    ros::NodeHandle m_node;

    /* Subscriber(s) */
    ros::Subscriber m_subscriber;

    /* Publisher(s) */
     ros::Publisher m_publisher_mapping_1;
     ros::Publisher m_publisher_mapping_2;
     ros::Publisher m_publisher_mapping_3;
     ros::Publisher m_publisher_mapping_4;

    /* Output variables */
    float m_x0, m_y0;   
    float m_a, m_b;  
    bool m_is_initialized;  

    /* Lidar detection parameters */
    // Front 0deg, left 90deg, right -90deg
    double MIN_RAD_LEFT   = (-120.0/180.0)*M_PI;
    double MAX_RAD_LEFT   = (-60.0/180.0)*M_PI;
    double MIN_RAD_RIGHT  = (+60.0/180.0)*M_PI;
    double MAX_RAD_RIGHT  = (+120.0/180.0)*M_PI;
};

#endif //BTOA_H
