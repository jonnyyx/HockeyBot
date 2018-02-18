#ifndef CLASSIFICATION_H
#define CLASSIFICATION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


/* Namespaces */
using namespace cv;
using namespace std;

/* Color IDs */ 
#define UNDEFINED 0
#define YELLOW 1
#define BLUE 2
#define GREEN 3

struct threshold_hsv{
    int lowH;
    int highH;
    int lowS;
    int highS;
    int lowV;
    int highV;
};

struct detected_object{
    int color; // Object color
    float x; // x coordinate relativ to the RGB Image
    float y; // y coordinate relativ to the RGB Image
    float ang; // angle between robot and object
    float dist;  // distance from robot to object relativ to Depth
}; 

/* roslaunch turtlebot_bringup 3dsensor.launch */
class Classification{
  public:
    Classification();
    ~Classification();
    vector<detected_object>* getRelevantObjects();
    int getTeamColor();
    void setTeamColor(int color);
	ros::NodeHandle m_node;

  private:
    void cameraCallBack(const sensor_msgs::ImageConstPtr& img_msg);
    void depthCallBack(const sensor_msgs::ImageConstPtr& depth_msg);
    void detectTeamColor();
    void detectObjects(const sensor_msgs::ImageConstPtr &msg, int object_color);
    float angleToObject(int x);
    float distanceToObject(int x, int y);
    void locateObjects();
  private:
    /* Node */
//    ros::NodeHandle m_node;

    /* Subscriber(s) */
    image_transport::ImageTransport m_img_transport;
    image_transport::Subscriber m_rgb_subscriber;
    ros::Subscriber m_depth_subscriber;


    /* Matrices necessary for the image processing */
    Mat m_img_hsv; // HSV Image
    Mat m_color_matrix; // Color specific Matrix
    cv_bridge::CvImagePtr m_img_ptr;
    cv_bridge::CvImagePtr m_depth_ptr;

    /* Image processing parameters */
    int m_erode;
    int m_dilate;
    int m_contour_threshold;

    /* Color scalar thresholds for yellow, blue and green*/
    threshold_hsv m_threshold_yellow;
    threshold_hsv m_threshold_blue;
    threshold_hsv m_threshold_green;

    /* Output variables */
    int m_team_color;  // 0: Default || 1: Blue || 2: Yellow
    vector<detected_object> m_objects_buffer;
};

#endif
