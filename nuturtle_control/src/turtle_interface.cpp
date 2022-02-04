#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "sensor_msgs/JointState.h"
#include "turtlelib/diff_drive.hpp"
class Message_handle{
public:
    ros::Publisher wheel_pub;
    ros::Publisher joint_pub;
    double radius;
    double track;
    turtlelib::DiffDrive dffrive;

    void velocity_callback(const geometry_msgs::Twist& msg);
    void sensor_callback(const nuturtlebot_msgs::SensorData& msg);


};
void Message_handle::velocity_callback(const geometry_msgs::Twist& msg){
    //here we only have the twist in 2D 
    turtlelib::Twist2D tw = {msg.angular.z,msg.linear.x,msg.linear.y};
    turtlelib::Vector2D wheel_vel = dffrive.IK_calculate(tw);
    nuturtlebot_msgs::WheelCommands wheel_cmd;
    wheel_cmd.left_velocity = wheel_vel.x;
    wheel_cmd.right_velocity = wheel_vel.y;
    this->wheel_pub.publish(wheel_cmd);
}

void Message_handle::sensor_callback(const nuturtlebot_msgs::SensorData& msg){
    double encoder_tick;
    ros::param::get("encoder_ticks_to_rad",encoder_tick);
    //wheel_pos correctly??
    turtlelib::Vector2D wheel_pos{msg.left_encoder*encoder_tick,msg.right_encoder*encoder_tick};
    //calculate the wheel_pos;
    dffrive.FK_calculate(wheel_pos,1);

    //publish on joint states?
}
int main(int argc, char ** argv){
    //init the node
    ros::init(argc, argv, "turtle_interface");

    Message_handle msgh;
    //get message
    double radius;
    double track;
    msgh.radius  =  ros::param::get("radius",radius);
    msgh.track = ros::param::get("track",track);
    //define the handler here
    ros::NodeHandle n;
    //subscribe to the cmd_vel
    ros::Subscriber cmd_sub = n.subscribe("cmd_vel",1000,&Message_handle::velocity_callback,&msgh);
    //publish the wheel command
    msgh.wheel_pub = n.advertise<nuturtlebot_msgs::WheelCommands>("cmd_vel", 1000);
    //subscribe to the sensor data
    ros::Subscriber sensor_sub = n.subscribe("sensor_data",1000,&Message_handle::sensor_callback,&msgh);
    //publish the joint states to provide the angle
    msgh.joint_pub = n.advertise<sensor_msgs::JointState>("joint_states",1000);
}
