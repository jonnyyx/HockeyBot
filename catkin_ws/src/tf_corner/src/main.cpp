#include "frames.h"

int main(int argc, char **argv){
    ros::init(argc,argv,"FramesNode");

    if (argc != 2){ROS_ERROR("need frame name as argument"); return -1;};
    string frame_name = argv[1];//string

    Frames corner1(frame_name);
    corner1.send();
    return 0;
}
