/// \file circle.cpp
/// \brief control the robot to make it drive in circle
/// SUBSCRIBERS:
///    blue/joint_states(sensor_msgs::sensor_msgs): the joint name, position and 
///           velocity of the wheels of the robot
/// PUBLISHERS:
///    odom(nav_msgs::Odometry): the configuration of the robot in odom frame
/// SERVICES:
///     control(nuturtle_control::control): give the radius and angular velocity of the robot
///     reverse(std_srvs::Empty): reverse the direction of the robot move
///     stop(std_srvs::Empty): stop the robota
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "nuturtle_control/control.h"
#include "turtlelib/diff_drive.hpp"

/// Static variables used by callbacks here

enum class State {INITIAL,STOP, RUN, REVERSE};

static State state = State::INITIAL;
static geometry_msgs::Twist twist;
int stop_flag = 0;
bool stop_callback(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
    // process message.  Maybe messages of this type
    // mean its supposed to go into STOP mode
    state = State::INITIAL;
    return true;
}

bool reverse_callback(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
    // process message.  Maybe messages of this type
    // mean its supposed to go into REVERSE mode
    state = State::REVERSE;
    twist.linear.x = -twist.linear.x;
    twist.angular.z = -twist.angular.z;
    return true;

}
bool control_callback(nuturtle_control::control::Request &req,nuturtle_control::control::Response &res)
{
    // process message.  Maybe messages of this type
    // mean its supposed to go into REVERSE mode
    state = State::RUN;

    double velocity = req.velocity;
    double radius = req.radius;
    twist.linear.x = velocity*radius;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = velocity;
    return true;  
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "circle");

    ros::NodeHandle n;

    // read parameters, create publishers/subscribers
    ros::ServiceServer control = n.advertiseService("control",control_callback);
    ros::ServiceServer reverse = n.advertiseService("reverse",reverse_callback);
    ros::ServiceServer stop = n.advertiseService("stop",stop_callback);

    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    double frequency;
    ros::param::get("~frequency",frequency);
    ros::Rate r(frequency);
    while(ros::ok())
    {
        switch(state)
        {   
            case State::INITIAL:
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.linear.z = 0;
                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = 0;
                cmd_pub.publish(twist);
            case State::STOP:
                if (!stop_flag){
                // do stop state stuff
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.linear.z = 0;
                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = 0;
                cmd_pub.publish(twist);
                stop_flag=1;}
                break;
            case State::REVERSE:
                // do go state stuff
                cmd_pub.publish(twist);
                break;
            case State::RUN:
                // do end state stuff
                cmd_pub.publish(twist);
                break;
            default:
                // should never get here
                throw std::logic_error("Invalid State");
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;   
}