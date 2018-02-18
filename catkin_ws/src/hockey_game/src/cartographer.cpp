#include "cartographer.h"
#include <std_msgs/Empty.h>


#define DEBUG 0

/*subscribes to scan topic published by the lidar
 * subscribes to the gmapping topic
 *
*/
Cartographer::Cartographer(){
    /* define subscriber(s) */
    m_subscriber = m_node.subscribe("scan", 1, &Cartographer::scanCallback, this);

    /* define Publisher(s) */
    m_publisher_mapping_1 = m_node.advertise<geometry_msgs::Twist>("corner1_topic", 100);
    m_publisher_mapping_2 = m_node.advertise<geometry_msgs::Twist>("corner2_topic", 100);
    m_publisher_mapping_3 = m_node.advertise<geometry_msgs::Twist>("corner3_topic", 100);
    m_publisher_mapping_4 = m_node.advertise<geometry_msgs::Twist>("corner4_topic", 100);
    
    /* Initialize output variables */
    m_a = 0.0;
    m_b = 0.0;
    m_x0 = 0.0;
    m_y0 = 0.0;
    m_is_initialized = false;
}


Cartographer::~Cartographer(){

}

void Cartographer::start(){
    ros::Rate rate(10);
    while (ros::ok() && !m_is_initialized){
        ros::spinOnce();
        rate.sleep();
    }
}
/*callback for measuring a/b values
 * only used as long as ab is not calculated
 *
*/
void Cartographer::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    if (DEBUG){
        ROS_INFO("Cartographer -- scanCallback: Started");
    }
    if (!m_is_initialized){
        delimitField(scan);
    } else {
        if (DEBUG){
            ROS_INFO("Cartographer -- scanCallback: Initialization finalized");
        }
    }
}
/*scans for cones on each side and calulates the resulting a/b values
 * set_is_active = true caluclating receiving a/b
 *
*/
void Cartographer::delimitField(const sensor_msgs::LaserScan_<std::allocator<void> >::ConstPtr &scan){

    if (DEBUG){
        ROS_INFO("Cartographer -- delimitField: starting ...");
    }

    cv::Point2f cones_coordinates_array[2*NB_CONES_AB];
    scanSideForCones(scan, MIN_RAD_LEFT, MAX_RAD_LEFT, cones_coordinates_array, LEFT_OFFSET_ARRAY);
    scanSideForCones(scan, MIN_RAD_RIGHT, MAX_RAD_RIGHT, cones_coordinates_array, RIGHT_OFFSET_ARRAY);

    /* Compute A/B Distances */
    //Left side
    float left_b = abs((cones_coordinates_array[0].y + cones_coordinates_array[1].y + cones_coordinates_array[2].y) / 3);
    float left_a = std::max(abs(cones_coordinates_array[0].x - cones_coordinates_array[1].x), abs(cones_coordinates_array[0].x - cones_coordinates_array[2].x));
    left_a = std::max(left_a, abs(cones_coordinates_array[1].x - cones_coordinates_array[2].x));

    //Right side
    float right_b = abs((cones_coordinates_array[0 + NB_CONES_AB].y + cones_coordinates_array[1 + NB_CONES_AB].y + cones_coordinates_array[2 + NB_CONES_AB].y) / 3);
    float right_a = std::max(abs(cones_coordinates_array[0 + NB_CONES_AB].x - cones_coordinates_array[1 + NB_CONES_AB].x), abs(cones_coordinates_array[0 + NB_CONES_AB].x - cones_coordinates_array[2 + NB_CONES_AB].x));
    right_a = std::max(right_a, abs(cones_coordinates_array[1 + NB_CONES_AB].x - cones_coordinates_array[2 + NB_CONES_AB].x));

    m_a = (left_a + right_a) /2;
    m_b = left_b + right_b;
    m_x0 = (abs(cones_coordinates_array[1].x) + abs(cones_coordinates_array[1 + NB_CONES_AB].x))/2;
    m_y0 = (abs(left_b) + abs(right_b))/2;

    geometry_msgs::Twist corner_left_front, corner_left_back, corner_right_front, corner_right_back;
    corner_left_back.linear.x = -m_x0;
    corner_left_back.linear.y = left_b;
    corner_right_back.linear.x = -m_x0;
    corner_right_back.linear.y =  -right_b;
    corner_left_front.linear.x = corner_left_back.linear.x + 3 * m_a;
    corner_left_front.linear.y = left_b;
    corner_right_front.linear.x = corner_right_back.linear.x + 3 * m_a;
    corner_right_front.linear.y =  -right_b;

    ros::Rate rate(10);
    ROS_INFO_STREAM("Cartographer -- delimitField: m_a = " << m_a);
    ROS_INFO_STREAM("Cartographer -- delimitField: m_b = " << m_b);

    
    if (DEBUG){
        ROS_INFO_STREAM("Cartographer -- delimitField: m_a = " << m_a);
        ROS_INFO_STREAM("Cartographer -- delimitField: m_b = " << m_b);
        ROS_INFO_STREAM("Cartographer -- delimitField: m_x0 = " << m_x0);
        ROS_INFO_STREAM("Cartographer -- delimitField: m_y0 = " << m_y0);
    }
    if(m_a > 0.5 && m_a <5 && m_b > 0.5 && m_b <7 ){
        m_is_initialized = true;

        m_publisher_mapping_1.publish(corner_left_back);
        //rate.sleep();
        m_publisher_mapping_2.publish(corner_left_front);
        //rate.sleep();
        m_publisher_mapping_3.publish(corner_right_back);
        //rate.sleep();
        m_publisher_mapping_4.publish(corner_right_front);
        //rate.sleep();
        // ros::spinOnce();
    }
}
/*scanning cones between anglemin and angle max
 *
*/
void Cartographer::scanSideForCones(const sensor_msgs::LaserScan::ConstPtr &scan, double minRad, double maxRad, cv::Point2f* cones_coordinates_array, int array_offset)
{
    int minIndex = ceil((minRad - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((maxRad- scan->angle_min) / scan->angle_increment);

    float range[NB_CONES_AB] = {DEF_RANGE_CONE, DEF_RANGE_CONE, DEF_RANGE_CONE};
    float ang[NB_CONES_AB] = {0, 0, 0};
    int ind[NB_CONES_AB] = {0, 0, 0};


    for(int curIndex = minIndex; curIndex <= maxIndex; curIndex++)
    {
        if(scan->ranges[curIndex] < range[0])
        {
            if(abs(curIndex - ind[0]) > MIN_WIDTH_CONES){
                range[2] = range[1];
                ind[2] = ind[1];

                range[1] = range[0];
                ind[1] = ind[0];
            }

            range[0] = scan->ranges[curIndex];
            ind[0] = curIndex;

        } else if(scan->ranges[curIndex] < range[1])
        {
            if (abs(curIndex - ind[1]) > MIN_WIDTH_CONES &&  abs(curIndex - ind[0]) > MIN_WIDTH_CONES){
                range[2] = range[1];
                ind[2] = ind[1];
            }
            if (abs(curIndex - ind[0]) > MIN_WIDTH_CONES){
                range[1] = scan->ranges[curIndex];
                ind[1] = curIndex;
            }

        } else if (scan->ranges[curIndex] < range[2])
        {
            if(abs(curIndex - ind[1]) > MIN_WIDTH_CONES && abs(curIndex - ind[0]) > MIN_WIDTH_CONES)
            {
                range[2] = scan->ranges[curIndex];
                ind[2] = curIndex;
            }
        }
    }

    /* output */
    for(int i = 0; i < NB_CONES_AB; i++)
    {
        ang[i] = (ind[i]*scan->angle_increment) + scan->angle_min;
        cv::Point2f dummy_point(cos(ang[i])*range[i], sin(ang[i])*range[i]);
        cones_coordinates_array[i + array_offset] = dummy_point;
        if (DEBUG){
            ROS_INFO_STREAM("Cartographer -- scanSideForCones: range[" << (i + array_offset) <<"] = " << range[i]);
            ROS_INFO_STREAM("Cartographer -- scanSideForCones: ang[" << (i + array_offset) <<"] = " << ang[i]);
            ROS_INFO_STREAM("Cartographer -- scanSideForCones: ind[" << (i + array_offset) <<"] = " << ind[i]);
            ROS_INFO_STREAM("Cartographer -- scanSideForCones: cones_coordinates_array[" << (i + array_offset) <<"] = {" << dummy_point.x << " ; " << dummy_point.y << "}");
        }
    }

    
}
/*returns the inititialized flag needed in the decider
 *
*/

bool Cartographer::getIsInitialized(){
    return m_is_initialized;
}

/*returns ab ratio
 *
*/
float Cartographer::getFieldRatio(){
    return m_a/m_b;
}


