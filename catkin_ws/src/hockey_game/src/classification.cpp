#include "classification.h"

#define DEBUG 0
/*subscribes camera topics
 * sets parameters for color detection
 *
*/
Classification::Classification():m_img_transport(m_node)
{
    // Double check order -> Crucial!!
    m_rgb_subscriber = m_img_transport.subscribe("camera/rgb/image_raw", 1, &Classification::cameraCallBack, this, image_transport::TransportHints("compressed"));
    m_depth_subscriber = m_node.subscribe("camera/depth/image_raw", 1, &Classification::depthCallBack, this);

    /* Optimal Parameters/Thresholds */
    m_erode = 4;
    m_dilate = 5;
    m_contour_threshold = 400;

    m_threshold_yellow = {20, 36, 90, 255, 120, 255};
    m_threshold_blue = {103, 135, 83, 255, 12, 161};
    m_threshold_green = {62, 102, 12, 255, 22, 255};

    /* Initialize variables */
    m_team_color = UNDEFINED;
}

Classification::~Classification()
{

}

/*callback reading rgb image and saves to them desktop, searechs for yellow blue green objects
*/
void Classification::cameraCallBack(const sensor_msgs::ImageConstPtr& img_msg)
{
    if (DEBUG){
        ROS_INFO("Classification -- cameraCallBack");
    }
    try{
        /* Convert RGB to HSV */
        m_img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        //cv_img_ptr->image.copyTo(m_img_rgb);
        cvtColor(m_img_ptr->image, m_img_hsv, COLOR_BGR2HSV);

        /* Filter out bottoom part of the image to avoid floor generated false detection */
        Rect upper(0 ,0, m_img_hsv.cols, m_img_hsv.rows/4);
        Rect lower(0, m_img_hsv.rows/2, m_img_hsv.cols, m_img_hsv.rows/2);
        rectangle(m_img_hsv, upper, Scalar(0,0,0), -1, 8, 0);
        rectangle(m_img_hsv, lower, Scalar(0,0,0), -1, 8, 0);
        

        /* Empty object buffer */
        m_objects_buffer.clear();

        /* Look for Blue, Yellow and Green objects in field view*/
        detectObjects(img_msg, YELLOW);
        detectObjects(img_msg, BLUE);
        detectObjects(img_msg, GREEN);


        if (DEBUG){
            ROS_INFO("CLASSIFICATION -- cameraCallBack: Image is being saved ...");
            imwrite("/home/ga38fed/Desktop/ImageA.png", m_img_ptr->image);
            ROS_INFO("CLASSIFICATION -- cameraCallBack: Image Saved");
        }
    } catch(cv_bridge::Exception& e){
        if (DEBUG){
            ROS_ERROR("Classification -- cameraCallBack (Exception): %s",e.what());
        }
        return;
    }
}
/*
 *callback for detph camera
*/
void Classification::depthCallBack(const sensor_msgs::ImageConstPtr& depth_msg)
{
    if (DEBUG){
        ROS_INFO("Classification -- depthCallBack");
    }

    try{
        m_depth_ptr = cv_bridge::toCvCopy(depth_msg);

        locateObjects();

        if (m_team_color == UNDEFINED) {
            detectTeamColor();
        } else {
            if (DEBUG){
                ROS_INFO_STREAM("Classification -- cameraCallBack: m_team_color = " << m_team_color);
            }
        }

    } catch(cv_bridge::Exception& e){
        if (DEBUG){
            ROS_ERROR("Classification -- depthCallBack (Exception): %s", e.what());
        }
        return;
    }
}

/*
 *detect team color by taking the color of neares object
*/
void Classification::detectTeamColor()
{
    bool output = false;
    detected_object dummy_object;
    if (DEBUG) {
        ROS_INFO_STREAM("Classification -- detectTeamColor: m_objects_buffer.size() = " << m_objects_buffer.size());
    }
    for (int i = 0; i < m_objects_buffer.size(); i++){
        if (DEBUG) {
            ROS_INFO_STREAM("Classification -- detectTeamColor: m_objects_buffer[" << i << "] = " << dummy_object.color << " || dist = " << dummy_object.dist);

        }
        if ((m_objects_buffer[i].color == BLUE || m_objects_buffer[i].color == YELLOW) && m_objects_buffer[i].dist > 0.2 && m_objects_buffer[i].dist < 2 ){
            if (!output){
                output = true;
                dummy_object = m_objects_buffer[i];
                if (DEBUG) {
                    ROS_INFO_STREAM("Classification -- detectTeamColor (detected): m_objects_buffer[" << i << "] = " << dummy_object.color << "dist = " << dummy_object.dist);
                }
            }else{
                if (dummy_object.dist > m_objects_buffer[i].dist){
                    ROS_INFO_STREAM("Classification -- detectTeamColor (detected): m_objects_buffer[" << i << "] = " << dummy_object.color << " || dist = " << dummy_object.dist);
                    dummy_object = m_objects_buffer[i];
                }
            }
        }
    }
    if (output){
        m_team_color = dummy_object.color;
    }

    if (DEBUG) {
        ROS_INFO_STREAM("Classification -- detectTeamColor: m_team_color = " << m_team_color);
    }
}

/*
 *detects and draws contours around yellow blue and green objects with a specific size, also draws a center point into the object
 * and puts the angles from the center to this center point into a vector for turning to the object
*/
void Classification::detectObjects(const sensor_msgs::ImageConstPtr &msg, int object_color)
{
    /* Color-based Image Filtering */
    Scalar color(255,0,0);
    switch (object_color){
    case BLUE:
        inRange(m_img_hsv, Scalar(m_threshold_blue.lowH, m_threshold_blue.lowS, m_threshold_blue.lowV),Scalar(m_threshold_blue.highH, m_threshold_blue.highS, m_threshold_blue.highV), m_color_matrix);
        break;
    case YELLOW:
        color.val[0] = 0;
        color.val[1] = 255;
        color.val[2] = 255;
        inRange(m_img_hsv, Scalar(m_threshold_yellow.lowH, m_threshold_yellow.lowS, m_threshold_yellow.lowV), Scalar(m_threshold_yellow.highH, m_threshold_yellow.highS, m_threshold_yellow.highV), m_color_matrix);
        break;
    case GREEN:
        color.val[0] = 0;
        color.val[1] = 255;
        color.val[2] = 0;
        inRange(m_img_hsv, Scalar(m_threshold_green.lowH, m_threshold_green.lowS, m_threshold_green.lowV),Scalar(m_threshold_green.highH, m_threshold_green.highS, m_threshold_green.highV), m_color_matrix);
        break;
    default:
        if (DEBUG){
            ROS_INFO("Classification -- detect_objects: Non valid color");
        }
        break;
    }

    /* Background/Foreground noise reduction*/
    // Background
    erode(m_color_matrix, m_color_matrix, getStructuringElement(2, Size(m_erode, m_erode), Point(-1,-1)));
    dilate(m_color_matrix, m_color_matrix, getStructuringElement(2, Size(m_dilate, m_dilate), Point(-1,-1)));
    // Foreground
    dilate(m_color_matrix, m_color_matrix, getStructuringElement(2, Size(m_dilate, m_dilate), Point(-1,-1)));
    erode(m_color_matrix, m_color_matrix, getStructuringElement(2, Size(m_erode, m_erode), Point(-1,-1)));
    
    /* Contour limitation */
    // Variable declaration
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    /* Contour finding */
    findContours(m_color_matrix, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    // Filter out non-relavant (small) Contours
    vector<vector<Point>> relevant_contours;

    for(int i = 0; i < contours.size(); i++){
        if(contourArea(contours[i]) > m_contour_threshold)
        {
            relevant_contours.push_back(contours[i]);
        }
    }

    vector<Rect> relevant_contours_boundaries(relevant_contours.size()); //Bounding Rectangles
    
    //Draw Contours into original
    for(int i = 0; i < relevant_contours.size(); i++)
    {
        relevant_contours_boundaries[i] = boundingRect(Mat(relevant_contours[i]));
        if(relevant_contours_boundaries[i].height > 40)
        {
            Point object_center((relevant_contours_boundaries[i].x + relevant_contours_boundaries[i].width/2), (relevant_contours_boundaries[i].y + relevant_contours_boundaries[i].height/2));
            rectangle(m_img_ptr->image, relevant_contours_boundaries[i], color, 2, 8, 0);
            circle(m_img_ptr->image, object_center, 5, color,2,8,0);

            /* Append detected objects to buffer */
            detected_object dummy_object;
            dummy_object.color = object_color;
            dummy_object.x = object_center.x;
            dummy_object.y = object_center.y;
            m_objects_buffer.push_back(dummy_object);
        }
    }
}
/*
 *is called to return the angle to the centerpoint of an object, parameters given by asus xtion pro
*/
float Classification::angleToObject(int x)
{
    float angle = x;
    angle = -(angle*58.0/640.0-29.0);
    return angle;
}
/*
 *returns the distance to an object in the depth image
*/
float Classification::distanceToObject(int x, int y)
{
    ushort distance = m_depth_ptr->image.at<ushort>(y,x); // A verifier
    return ((float)distance)/1000.0;
}
/*
 *calculates the angles by calling angletoobject and distancetoobject and saves the objects in a vector
*/
void Classification::locateObjects(){
    if (DEBUG){
        ROS_INFO_STREAM("Classification -- locateOjects: m_objects_buffer.size = " << m_objects_buffer.size());
    }
    for (int i=0; i < m_objects_buffer.size(); i++){
        m_objects_buffer[i].ang = angleToObject(m_objects_buffer[i].x);
        m_objects_buffer[i].dist = distanceToObject(m_objects_buffer[i].x, m_objects_buffer[i].y);

        if (DEBUG){
            ROS_INFO_STREAM("Classification -- locateObjects m_objects_buffer[i].color = " << m_objects_buffer[i].color);
            ROS_INFO_STREAM("Classification -- locateObjects m_objects_buffer[i].x = " << m_objects_buffer[i].x);
            ROS_INFO_STREAM("Classification -- locateObjects m_objects_buffer[i].y = " << m_objects_buffer[i].y);
            ROS_INFO_STREAM("Classification -- locateObjects m_objects_buffer[i].dist = " << m_objects_buffer[i].dist);
            ROS_INFO_STREAM("Classification -- locateObjects  m_objects_buffer[i].ang = " <<  m_objects_buffer[i].ang);
        }

    }
} 

/*
 *return the detected team color
*/
int Classification::getTeamColor(){
    return m_team_color;
}
/*
 *sets the received team by angelina
*/
void Classification::setTeamColor(int color){
    m_team_color = color;
}
/*
 *returns the object buffer
*/
vector<detected_object>* Classification::getRelevantObjects(){
    return &m_objects_buffer;
}
