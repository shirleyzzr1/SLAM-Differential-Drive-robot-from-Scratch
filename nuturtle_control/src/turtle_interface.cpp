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
    
    turtlelib::DiffDrive diffdrive;
    sensor_msgs::JointState joint_state;
    nuturtlebot_msgs::WheelCommands wheel_cmd;

    void velocity_callback(const geometry_msgs::Twist& msg);
    void sensor_callback(const nuturtlebot_msgs::SensorData& msg);

};
void Message_handle::velocity_callback(const geometry_msgs::Twist& msg){
    //here we only have the twist in 2D 
    turtlelib::Twist2D tw = {msg.angular.z,msg.linear.x,msg.linear.y};

    turtlelib::Vector2D wheel_vel = diffdrive.IK_calculate(tw);

    this->wheel_cmd.left_velocity = wheel_vel.x;
    this->wheel_cmd.right_velocity = wheel_vel.y;
    this->wheel_pub.publish(wheel_cmd);
}

void Message_handle::sensor_callback(const nuturtlebot_msgs::SensorData& msg){
    double encoder_ticks_to_rad,motor_cmd_to_radsec;
    ros::param::get("encoder_ticks_to_rad",encoder_ticks_to_rad);
    ros::param::get("motor_cmd_to_radsec",motor_cmd_to_radsec);
    
    sensor_msgs::JointState joint_state;

    joint_state.position.push_back(msg.left_encoder* 2 * turtlelib::PI/encoder_ticks_to_rad);
    joint_state.position.push_back(msg.right_encoder* 2 * turtlelib::PI/encoder_ticks_to_rad);

    joint_state.velocity.push_back(msg.left_encoder * motor_cmd_to_radsec);
    joint_state.velocity.push_back(msg.right_encoder * motor_cmd_to_radsec);

    joint_state.header.stamp = ros::Time::now();
    joint_pub.publish(joint_state);


}
int main(int argc, char ** argv){
    //init the node
    ros::init(argc, argv, "turtle_interface");

    double radius,track;
    ros::param::get("/wheel_radius",radius);
    ros::param::get("/track_width",track);

    Message_handle msgh;

    msgh.diffdrive.set_param(radius,track);
    //define the handler here
    ros::NodeHandle n;
    //subscribe to the cmd_vel
    ros::Subscriber cmd_sub = n.subscribe("cmd_vel",1000,&Message_handle::velocity_callback,&msgh);
    //publish the wheel command
    msgh.wheel_pub = n.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", 1000);
    //subscribe to the sensor data
    ros::Subscriber sensor_sub = n.subscribe("sensor_data",1000,&Message_handle::sensor_callback,&msgh);
    //publish the joint states to provide the angle
    msgh.joint_pub = n.advertise<sensor_msgs::JointState>("joint_states",1000);

    double rate = 50;
    ros::Rate r(rate); // 50 hz by default

    while(ros::ok()){
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.push_back("red/wheel_left_joint");
        joint_state.name.push_back("red/wheel_right_joint");
        joint_state.velocity.push_back(msgh.wheel_cmd.left_velocity);
        joint_state.velocity.push_back(msgh.wheel_cmd.right_velocity);

        turtlelib::Vector2D wheel =  {(double)msgh.wheel_cmd.left_velocity/rate,(double)msgh.wheel_cmd.right_velocity*(1/rate)};
        msgh.diffdrive.wheel_pos()+=wheel;
        joint_state.position.push_back(msgh.diffdrive.wheel_pos().x);
        joint_state.position.push_back(msgh.diffdrive.wheel_pos().y);
        msgh.joint_pub.publish(joint_state);
        ros::spinOnce();
        r.sleep();
    }
    return 0;

    

}
