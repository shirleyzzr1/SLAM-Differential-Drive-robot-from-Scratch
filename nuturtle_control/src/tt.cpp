#include "ros/ros.h"
#include "turtlelib/diff_drive.hpp"
int main(int argc, char ** argv){
    ros::init(argc, argv, "tt");
    turtlelib::DiffDrive diffdrive;
    diffdrive.FK_calculate({-10,10});
    return 0;
}