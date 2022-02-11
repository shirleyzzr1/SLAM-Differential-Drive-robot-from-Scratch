/// \file turtle_interface.cpp
/// \brief subscribe to cmd_vel and sensor data and publish joint_states and wheel_velocity message
///
/// PARAMETERS:
///     radius(double): the wheel radius of the robot
///     track(double): the body track of the robot
///     encoder_ticks_to_rad(double): converts encoder tiacks to radians
///     motor_cmd_to_radsec(double): converts between ticks and rad/sec
/// SUBSCRIBES:
///     cmd_sub(geometry_msgs::Twist): the user given body twist
///     sensor_sub(nuturtlebot_msgs::SensorData): the sensor data 
/// PUBLISHERS:
///     wheel_pub(nuturtlebot::WheelCommands): publish the wheel velocity
///     joint_pub(sensor_msgs::JointState): publish the joint state to the odometry
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "sensor_msgs/JointState.h"
#include "turtlelib/diff_drive.hpp"
#include <tf2_ros/transform_broadcaster.h>

class Message_handle{
public:
    ros::Publisher wheel_pub;
    ros::Publisher joint_pub;

    ros::Subscriber cmd_sub;
    ros::Subscriber sensor_sub;
    
    turtlelib::DiffDrive diffdrive;
    sensor_msgs::JointState joint_state;
    nuturtlebot_msgs::WheelCommands wheel_cmd;

    double max_wheel_vel;
    double left_encoder;
    double right_encoder;

    double encoder_ticks_to_rad;
    double motor_cmd_to_radsec;


    void velocity_callback(const geometry_msgs::Twist& msg);
    void sensor_callback(const nuturtlebot_msgs::SensorData& msg);

};
/// \brief subscibe to the /cmd_vel and publish the /wheel_cmd topic
/// \param msg: the body twist command
void Message_handle::velocity_callback(const geometry_msgs::Twist& msg){
    //here we only have the twist in 2D 
    turtlelib::Twist2D tw = {msg.angular.z,msg.linear.x,msg.linear.y};

    turtlelib::Vector2D wheel_vel = diffdrive.IK_calculate(tw);

    this->wheel_cmd.left_velocity = wheel_vel.x / this->motor_cmd_to_radsec;
    this->wheel_cmd.right_velocity = wheel_vel.y / this->motor_cmd_to_radsec;

    if(this->wheel_cmd.left_velocity>256){
        this->wheel_cmd.left_velocity = 256;
    }
    if (this->wheel_cmd.left_velocity<-256)
    {
        this->wheel_cmd.left_velocity = -256;
    }
    if (this->wheel_cmd.right_velocity>256)
    {
        this->wheel_cmd.right_velocity = 256;
    }
    if (this->wheel_cmd.right_velocity<-256)
    {
        this->wheel_cmd.right_velocity = -256;
    }
    
    this->wheel_pub.publish(this->wheel_cmd);
}
/// \brief subscribe to the /sensor_data and publish the /joint_states topic
/// \param msg: the data sensor from other source(encoder, ...)
void Message_handle::sensor_callback(const nuturtlebot_msgs::SensorData& msg){
    this->left_encoder = msg.left_encoder;
    this->right_encoder = msg.right_encoder;
    sensor_msgs::JointState joint_state;
    joint_state.name.push_back("blue/wheel_left_joint");
    joint_state.name.push_back("blue/wheel_right_joint");
    joint_state.position.push_back(this->left_encoder* 2 * turtlelib::PI/encoder_ticks_to_rad);
    joint_state.position.push_back(this->right_encoder* 2 * turtlelib::PI/encoder_ticks_to_rad);

    joint_state.velocity.push_back(this->diffdrive.wheel_vel().x);
    joint_state.velocity.push_back(this->diffdrive.wheel_vel().y);   

    joint_state.header.stamp = ros::Time::now();
    this->joint_pub.publish(joint_state);

}
int main(int argc, char ** argv){
    //init the node
    ros::init(argc, argv, "turtle_interface");

    double radius,track;
    ros::param::get("/wheel_radius",radius);
    ros::param::get("/track_width",track);

    Message_handle msgh;

    msgh.diffdrive.set_param(radius,track);
    //calculate max_wheel_vel given the max_trans_velocity is 0.22m/s
    turtlelib::Twist2D max_trans = {0,0.22,0};
    msgh.diffdrive.IK_calculate(max_trans);
    msgh.max_wheel_vel = abs(msgh.diffdrive.wheel_vel().x);
    //reset the diffdive wheel_velocity;
    msgh.diffdrive.set_wheel_vel({0,0});

    double encoder_ticks_to_rad,motor_cmd_to_radsec;
    ros::param::get("encoder_ticks_to_rad",encoder_ticks_to_rad);
    ros::param::get("motor_cmd_to_radsec",motor_cmd_to_radsec);
    msgh.encoder_ticks_to_rad = encoder_ticks_to_rad;
    msgh.motor_cmd_to_radsec = motor_cmd_to_radsec;
    //define the handler here
    ros::NodeHandle n;
    //subscribe to the cmd_vel
    msgh.cmd_sub = n.subscribe("cmd_vel",1000,&Message_handle::velocity_callback,&msgh);
    //publish the wheel command
    msgh.wheel_pub = n.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", 1000);
    //subscribe to the sensor data
    msgh.sensor_sub = n.subscribe("sensor_data",1000,&Message_handle::sensor_callback,&msgh);
    //publish the joint states to provide the angle
    msgh.joint_pub = n.advertise<sensor_msgs::JointState>("blue/joint_states",1000);

    double rate = 50;
    ros::Rate r(rate); // 50 hz by default

    while(ros::ok()){


        ros::spinOnce();
        r.sleep();
    }
        return 0;

    

}
