#include <ros/ros.h>
#include <QtGui>
#include <iostream>
#include "decider.h"

int main(int argc, char **argv){

    QApplication app(argc, argv);
    ros::init(argc,argv,"Decider");
    Decider decider(&app);
    decider.main();
    return 0;
}
