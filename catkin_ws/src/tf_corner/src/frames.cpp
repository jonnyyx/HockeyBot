#include "frames.h"

Frames::Frames(string frame_id)
:
      _frames_broad(new tf::TransformBroadcaster),
      _frames_listen(new tf::TransformListener),
      _frame_id(frame_id),
      _trans_stamped(tf::StampedTransform()),
      _calibrationDone(false)
{
    ros::NodeHandle _node;
    _frames_sub = _node.subscribe(_frame_id+"_topic", 1, &Frames::calibrate, this);
}

Frames::~Frames()
{
    delete _frames_broad;
    delete _frames_listen;
}

void Frames::calibrate(const geometry_msgs::Twist& t)
{
    ROS_INFO("calibrate frame");
    _frames_listen->waitForTransform("map", "base_footprint", ros::Time::now(), ros::Duration(5.0));
    _frames_listen->lookupTransform ("map", "base_footprint", ros::Time(0), _trans_stamped);
    double x = _trans_stamped.getOrigin().x();
    double y = _trans_stamped.getOrigin().y();
    ROS_INFO("Frames::calibrate -------------x= %f", x);
    ROS_INFO("Frames::calibrate -------------y= %f", y);
    x = x + t.linear.x;
    y = y + t.linear.y;
    ROS_INFO("Frames::calibrate -------------x= %f", x);
    ROS_INFO("Frames::calibrate -------------y= %f", y);
    tf::Quaternion q1 = _trans_stamped.getRotation();
    tf::Quaternion q2;
    q2.setRPY(0, 0, t.angular.z);
    _trans_stamped.setOrigin(tf::Vector3(x, y, _trans_stamped.getOrigin().z()));
    _trans_stamped.setRotation(q2*q1);
    _trans_stamped.child_frame_id_=_frame_id;
    _trans_stamped.frame_id_="map";
    _calibrationDone =true;
}

void Frames::send()
{
    ros::Rate rate(10.0);
    while(ros::ok()){
        if(_calibrationDone==true){
        //ROS_INFO("send-attempt");
        _trans_stamped.stamp_=ros::Time::now();
        _frames_broad->sendTransform(_trans_stamped);
        rate.sleep();
        }
        ros::spinOnce();
    }
}
