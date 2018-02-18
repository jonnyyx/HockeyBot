#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "classification.h"
#include <math.h>
#include <tf/transform_listener.h>

#define Radius_Robot 0.2

struct goal{
    float x_mid;
    float y_mid;
    float x_max;
    float x_min;
    float y_max;
    float y_min;
};


class Navigation {
public:
    Navigation(Classification *cl);
    ~Navigation();

    tf::TransformListener* xy_listener;
    tf::StampedTransform xy_transformed;
    ros::Publisher cmdPub; //drive Robot
    ros::NodeHandle naviNode;
    void nextPuck();
    void setAB(float a, float b);
    void setIsActive(bool is_active);
    void setLocked(bool is_locked);
    bool getLocked();
    double* get_robot_position();


    detected_object getTargetObject();
    bool lockedTarget(int color);
    bool turnRobot(float target_angle, bool is_dynamic_speed);
    bool moveRobot(float distance, bool is_forward);
    vector<float> *getGoalCoordinates();


private:
    double m_xy_buffer[4];
    detected_object m_target_object;
    Classification* m_classificator;
    bool m_is_locked;
    float m_a, m_b; //get value from angelina !!!!!
    int m_counter;
    bool m_is_active;

private: //moveRobot related variable(s)
    float driven_distance;
    double start_position[2];

private: //turnRobot related variable(s)
    float current_angle;
    float dummy_angle;

    vector<float> output_vector;

};

#endif
