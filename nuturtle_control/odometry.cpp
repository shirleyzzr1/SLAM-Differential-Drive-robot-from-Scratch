#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>

int main(){
    //init the node here
    ros::init(argc, argv, "odometry");

    ros::NodeHandle n;
    std::string body_id,wheel_left,wheel_right; 
    if (!n.getParam("body_id", body_id))
    {
        ROS_DEBUG_STREAM("not initialize the body id"); 
        ros::shutdown();
    }
    if (!n.getParam("left_whee_joint", wheel_left))
    {
        ROS_DEBUG_STREAM("not specify the left_wheel_joint"); 
        ros::shutdown();
    }
    if (!n.getParam("right_whee_joint", wheel_right))
    {
        ROS_DEBUG_STREAM("not specify the right_wheel_joint"); 
        ros::shutdown();
    }
    n.param<std::string>("odom_id", s, "odom");
    




}