#include "navigation.h"

#define DEBUG 0

Navigation::Navigation(Classification *cl): m_classificator(cl), xy_listener(new tf::TransformListener)
{
    cmdPub = naviNode.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);
    m_a = 1.2; // default values, correct it with value out of angelina
    m_b = 3.0; // default values, correct it with value out of angelina
    m_is_locked = false;
    m_is_active = true;
}

Navigation::~Navigation()
{
}

/*
 *function to turn the robot for a given angle target_angle, dynamic speed = true -> slow down when getting near this angle
*/
bool Navigation::turnRobot(float target_angle, bool is_dynamic_speed)
{
    geometry_msgs::Twist msg;
    bool output = false;
    if(!m_is_locked){
        get_robot_position();
        current_angle = m_xy_buffer[2];
        dummy_angle = 0.0;

        if (target_angle < 0) {
            if(current_angle + target_angle < 0.0){
                dummy_angle = 360.0 + (current_angle + target_angle);
            }else {
                dummy_angle = current_angle + target_angle;
            }
        } else {
            dummy_angle = fmod((current_angle + target_angle), 360.0);
        }
        m_is_locked = true;
    }

    if (m_is_active){
        /* Perform rotation */
        if (fabs(dummy_angle - current_angle) > 10.0){
            if (target_angle > 0.0){
                msg.angular.z = 0.6;
                msg.linear.x = 0.0;
            } else {
                msg.angular.z = -0.6;
                msg.linear.x = 0.0;
            }
            cmdPub.publish(msg);
        } else if((fabs(dummy_angle - current_angle) <= 10) && (fabs(dummy_angle - current_angle) > 0.8)){
            if (target_angle > 0.0){
                msg.angular.z = is_dynamic_speed ? 0.15 : 0.25;
                msg.linear.x = 0.0;
            } else {
                msg.angular.z = is_dynamic_speed ? -0.15 : -0.25;
                msg.linear.x = 0.0;
            }
            cmdPub.publish(msg);
        }else{
            msg.angular.z = 0.0;
            msg.linear.x = 0.0;
            cmdPub.publish(msg);
            output = true;
            m_is_locked = false;
        }

        get_robot_position();
        current_angle = m_xy_buffer[2];
    }else{
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        cmdPub.publish(msg);
        m_is_locked = false;
    }
    return output;

}

/*
 *drives forward/backward depending on is_forward for the distance given
 * returns true if distance is driven else false
*/
bool Navigation::moveRobot(float distance, bool is_forward){
    geometry_msgs::Twist msg;
    bool output = false;
    if (!m_is_locked){
        driven_distance = 0.0;
        get_robot_position();
        start_position[0] = m_xy_buffer[0];
        start_position[1] = m_xy_buffer[1];
        m_is_locked = true;
    }
    if (m_is_active){
        driven_distance = fabs(sqrt((start_position[0] - m_xy_buffer[0]) * (start_position[0] - m_xy_buffer[0]) + (start_position[1] - m_xy_buffer[1]) * (start_position[1] - m_xy_buffer[1])));
        if(distance > driven_distance){
            msg.linear.x = is_forward ? 0.3 : -0.3;
            msg.angular.z = 0.0;
            cmdPub.publish(msg);
        } else{
            if (DEBUG){
                ROS_INFO_STREAM("Navigation -- moveRobot: Stopped");
            }
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            cmdPub.publish(msg);
            output = true;
            m_is_locked = false;
        }
        get_robot_position();
        if (DEBUG){
            ROS_INFO_STREAM("Navigation -- moveRobot: m_x= " << m_xy_buffer[0] << " m_y: " << m_xy_buffer[1] << " m_angle: " << m_xy_buffer[2] << "}");
        }
    }else{
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        cmdPub.publish(msg);
        m_is_locked = false;
    }
    return output;
}

/*
 *locks an object of a given color blue or yellow if no obect is found in the field of view of the camera or if the distance to the
 * object is < 0.2 or > 2 the function will return 0 else if an object in range is found it returns true and the object is saved to
 * m_target_object
*/
bool Navigation::lockedTarget(int color)

{
    vector<detected_object> objects_buffer = *m_classificator->getRelevantObjects();
    bool output = false;

    if (!m_is_locked){
        // get array -> differentiate based on color get pointer to nearest target if empty/no target return -1
        for (int i = 0; i < objects_buffer.size(); i++){
            if (objects_buffer[i].color == color && objects_buffer[i].dist > 0.2 && objects_buffer[i].dist < 2 ){
                if (!output){
                    output = true;
                    m_target_object = objects_buffer[i];
                }else{
                    if (m_target_object.dist > objects_buffer[i].dist){
                        m_target_object = objects_buffer[i];
                    }
                }
            }
        }
        if (m_target_object.dist < 0.2|| m_target_object.dist > 2.0){
            output = false;
        }

    }
    if (output) {
        ROS_INFO_STREAM("Navigation -- lockedTarget: Target Locked");
    }
    return output;
}

/*
 *returns the robot_position given gmapping
*/
double* Navigation::get_robot_position(){
    try{
        xy_listener->waitForTransform("corner1", "base_footprint", ros::Time::now(), ros::Duration(10.0));
        xy_listener->lookupTransform("corner1", "base_footprint",ros::Time(0), xy_transformed);
        m_xy_buffer[0] = xy_transformed.getOrigin().x();
        m_xy_buffer[1] = - xy_transformed.getOrigin().y();

        //ROS_INFO_STREAM("Navigation -- get_robot_position: m_x= " << m_xy_buffer[0] << " m_y: " << m_xy_buffer[1] << " m_angle: " << m_xy_buffer[2] << "}");

        tf::Quaternion q =  xy_transformed.getRotation();
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        m_xy_buffer[2] = tf::getYaw(xy_transformed.getRotation())*180.0/M_PI;
        m_xy_buffer[2] = (m_xy_buffer[2] < 0) ? (360 + m_xy_buffer[2]) : fabs(m_xy_buffer[2]);
        return m_xy_buffer;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return 0;
    }

}

/*
 *calulates the goal coordinates using ab values
*/
vector<float>* Navigation::getGoalCoordinates()
{
    output_vector.clear();

    goal target_goal;
    target_goal.x_mid = 3 * m_a - m_a*0.25;
    target_goal.y_mid = m_b - m_b*0.5;
    target_goal.x_max = 3 * m_a - m_a / 4.0;
    target_goal.x_min = 3 * m_a - m_a / 2.0;
    target_goal.y_min = m_b / 3.0;
    target_goal.y_max = m_b - m_b /3.0;


    //get_robot_position();
    float current_angle = m_xy_buffer[2];
    float dummy_angle;
    float YY = fabs(m_xy_buffer[1] - target_goal.y_mid); //nenner
    float XX = fabs(m_xy_buffer[0] - target_goal.x_mid); //zähler
    float y_robo = m_xy_buffer[1];
    float x_robo = m_xy_buffer[0];

    float alpha = atan(YY/XX)*180/M_PI;

    if (DEBUG) {
        ROS_INFO_STREAM("Navigation -- moveToGoal: alpha" << alpha);
    }
    float distancetogoal = sqrt(YY * YY + XX * XX);


    if(x_robo <= target_goal.x_min){
        if(y_robo >= target_goal.y_max - 0.3){
            if(current_angle <= 180){
                dummy_angle = alpha - current_angle;
            }else{
                dummy_angle = 360 - current_angle + alpha;
            }
        }else if(y_robo <= target_goal.y_min + 0.3){
            if(current_angle <= 180){
                dummy_angle = -(current_angle + alpha);
            }else{
                dummy_angle = 360 - current_angle - alpha;
            }
        }else{
            if(current_angle <= 180){
                dummy_angle = -(current_angle);
            }else{
                dummy_angle = 360 - current_angle;
            }
        }
    }else if(x_robo >= target_goal.x_max){
        if(y_robo >= target_goal.y_max - 0.3){
            if(current_angle <= 180){
                dummy_angle = 180 - alpha - current_angle;
            }else{
                dummy_angle = 360 - current_angle - alpha;
            }
        }else if(y_robo <= target_goal.y_min + 0.3){
            if(current_angle <= 180){
                dummy_angle = 180 + alpha - current_angle;
            }else{
                dummy_angle = current_angle - 180 - alpha;
            }
        }else{
            if(current_angle <= 180){
                dummy_angle = 180 -current_angle;
            }else{
                dummy_angle = (180 - current_angle);
            }
        }
    }else{
        if(y_robo >= target_goal.y_max - 0.3){
            if(current_angle <= 180){
                dummy_angle = 90 - current_angle;
            }else{
                dummy_angle = 360 - current_angle +90;
            }
        }else if(y_robo <= target_goal.y_min + 0.3){
            if(current_angle <= 180){
                dummy_angle = 270 - current_angle;
            }else{
                dummy_angle = 270 - current_angle;
            }
        }
    }
    if (DEBUG) {
        ROS_INFO_STREAM("Navigation -- moveToGoal: dummy_angle" << dummy_angle);
        ROS_INFO_STREAM("Navigation -- moveToGoal: m_x= " << m_xy_buffer[0] << " m_y: " << m_xy_buffer[1] << " m_angle: " << m_xy_buffer[2] << "}");
    }
    output_vector.push_back(distancetogoal - 0.5);
    output_vector.push_back(dummy_angle);
    return &output_vector;
}


/*
 *logic to get next puck after scoring a goal
*/
/*
void Navigation::nextPuck(){
    //moveTotarget();
    //moveRobot(0.3, false);
    float dummy_angle = m_xy_buffer[2];
    turnRobot(180- dummy_angle, true);
    moveRobot(2*m_a - 0.3, true);
    if (true){
        ROS_INFO_STREAM("Navigation -- second Puck");
    }
    //moveTotarget();
    //moveRobot(0.3, false);
    dummy_angle = m_xy_buffer[2];
    turnRobot(180- dummy_angle, true);
    moveRobot(2*m_a - 0.3, true);

    if (true){
        ROS_INFO_STREAM("Navigation -- third Puck");
    }

    //moveTotarget();
    moveRobot(0.3, false);
    dummy_angle = m_xy_buffer[2];
    turnRobot(180- dummy_angle, true);
    moveRobot(2*m_a - 0.3, true);
}
*/

/*
 *sets the ab values given by angline
*/
void Navigation::setAB(float a, float b){
   // m_a = 1.2;
   // m_b = 3.0;
    m_a = a;
    m_b = b;

}
void Navigation::setIsActive(bool is_active){
    m_is_active = is_active;
}

detected_object Navigation::getTargetObject(){
    return m_target_object;
}

void Navigation::setLocked(bool is_locked){
    m_is_locked = is_locked;
}

bool Navigation::getLocked(){
    return m_is_locked;
}
