#ifndef FRAMES_H
#define FRAMES_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string>
#include <geometry_msgs/Twist.h>

using namespace std;

class Frames{

public:
    Frames(string frame_id);
    ~Frames();
    /**
      calibrate takes relative distance of corner from current base_footprint frame
     */
    void calibrate(const geometry_msgs::Twist &t);
    void send();

private:
    ros::NodeHandle _node;
    tf::TransformBroadcaster* _frames_broad;
    tf::TransformListener* _frames_listen;
    ros::Subscriber _frames_sub;
    string _frame_id;
    tf::StampedTransform _trans_stamped;
    bool _calibrationDone;
    
};

#endif
