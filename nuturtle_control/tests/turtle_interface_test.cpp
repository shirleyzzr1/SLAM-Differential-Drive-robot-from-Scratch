#include<catch_ros/catch.hpp>
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"

def wheel_velocity_callback(const nuturtlebot_msgs::WheelCommands& msg){
    
}
TEST_CASE("pure_translation"){
    ros::NodeHandle n;
    ros::Publisher twist_pub= n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    geometry_msgs::Twist twist;
    twist.pos.linear.x = 1;
    twist.pos.linear.y = 2;
    twist_pub.publish(twist);
    ros::Subscriber wheel_sub = n.subscribe("wheel_cmd",1000 ,wheel_velocity_callback);



}