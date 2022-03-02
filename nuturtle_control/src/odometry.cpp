/// \file odometry.cpp
/// \brief subcribe to joint states message and transform between the odom frame 
/// and the robot base_foorprint frame
/// SUBSCRIBERS:
///    blue/joint_states(sensor_msgs::sensor_msgs): the joint name, position and 
///           velocity of the wheels of the robot
/// PUBLISHERS:
///    odom(nav_msgs::Odometry): the configuration of the robot in odom frame
/// SERVICES:
///    set_pos(nusim::pose): set the position of the robot in odom frame
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "nusim/pose.h"
#include "turtlelib/diff_drive.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class Message_handle{
public:
    std::string body_id,odom_id,wheel_left,wheel_right; 
    ros::Publisher odom_pub;
    ros::ServiceServer set_pose;
    ros::Publisher path_pub;

    std::vector<geometry_msgs::PoseStamped> poses;

    turtlelib::DiffDrive diffdrive;
    double first_joint_flag;
    void transform();
    bool set_pose_callback(nusim::pose::Request &req,nusim::pose::Response &res);
    void joint_state_callback(const sensor_msgs::JointState& msg);

};

/// \brief broadcast the transform from world frame to robot red/base_footprint frame
/// x,y,theta input the pose of the robot relative to the world frame
void Message_handle::transform(){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = this->odom_id ;
    transformStamped.child_frame_id = this->body_id ;

    //first set the translation
    transformStamped.transform.translation.x = this->diffdrive.body_pos().translation().x;
    transformStamped.transform.translation.y = this->diffdrive.body_pos().translation().y;
    transformStamped.transform.translation.z = 0.0;
    //set the rotation og the robot
    tf2::Quaternion q;
    q.setRPY(0, 0, this->diffdrive.body_pos().rotation());
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
}
/// \brief subscribe to the joint states and publish the odometry message
/// \param msg: the joint state pos, velocity and name
void Message_handle::joint_state_callback(const sensor_msgs::JointState& msg){

    nav_msgs::Odometry odom;
    odom.header.frame_id = this->odom_id;
    odom.header.stamp = ros::Time::now();
    odom.child_frame_id = this->body_id;
    if(msg.velocity.size()==0) turtlelib::Vector2D wheel_vel{0,0};
    else{turtlelib::Vector2D wheel_vel{msg.velocity[0],msg.velocity[1]};}
    turtlelib::Vector2D wheel_pos{msg.position[0],msg.position[1]};
    //avoid sudden change in pos at the very start
    if(this->first_joint_flag){
        turtlelib::Transform2D trans;
        diffdrive.set_body_pos(trans);
        diffdrive.set_wheel_pos(wheel_pos);
        this->first_joint_flag = 0;
    }
    this->diffdrive.FK_calculate(wheel_pos);
    turtlelib::Transform2D trans = {{this->diffdrive.body_pos().translation().x,this->diffdrive.body_pos().translation().y},\
                turtlelib::normalize_angle(this->diffdrive.body_pos().rotation())};
    this->diffdrive.set_body_pos(trans);
    geometry_msgs::Pose pos;
    pos.position.x = this->diffdrive.body_pos().translation().x;
    pos.position.y = this->diffdrive.body_pos().translation().y;
    pos.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, this->diffdrive.body_pos().rotation());
    pos.orientation.x = q.x();
    pos.orientation.y = q.y();
    pos.orientation.z = q.z();
    pos.orientation.w = q.w();
    odom.pose.pose = pos;

    geometry_msgs::Twist twist;
    odom.twist.twist.linear.x = this->diffdrive.bodytwist().xdot;
    odom.twist.twist.linear.y = this->diffdrive.bodytwist().ydot;
    odom.twist.twist.angular.z = this->diffdrive.bodytwist().thetadot;

    odom_pub.publish(odom);

    nav_msgs::Path path;
    path.header.frame_id = this->odom_id;
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = this->odom_id;
    pose.pose.position.x = this->diffdrive.body_pos().translation().x;
    pose.pose.position.y = this->diffdrive.body_pos().translation().y;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    this->poses.push_back(pose);
    path.poses = poses;
    this->path_pub.publish(path);

}
/// \brief set the pose of the robot
/// \param req:theta,x,y
/// \param res:empty
bool Message_handle::set_pose_callback(nusim::pose::Request &req,nusim::pose::Response &res){
    turtlelib::Vector2D pos = {req.x,req.y};
    diffdrive.set_body_pos(turtlelib::Transform2D(pos,req.theta));
    return true;
}
int main(int argc, char ** argv){
    //init the node here
    ros::init(argc, argv, "odometry");

    double radius,track;
    ros::param::get("/wheel_radius",radius);
    ros::param::get("/track_width",track);

    Message_handle msgh;
    msgh.diffdrive.set_param(radius,track);
    msgh.first_joint_flag = 1;
    ros::NodeHandle n;
    //if not getting the node, shutdown
    if (!n.getParam("/body_id", msgh.body_id))
    {
        ROS_INFO_STREAM("not initialize the body id"); 
        ros::shutdown();
    }
    if (!n.getParam("/wheel_left", msgh.wheel_left))
    {
        ROS_INFO_STREAM("not specify the left_wheel_joint"); 
        ros::shutdown();
    }
    if (!n.getParam("/wheel_right", msgh.wheel_right))
    {
        ROS_INFO_STREAM("not specify the right_wheel_joint"); 
        ros::shutdown();
    }
    n.param<std::string>("/odom_id", msgh.odom_id, "odom");

    //subscribe to the cmd_vel
    ros::Subscriber joint_sub = n.subscribe("blue/joint_states",1000, &Message_handle::joint_state_callback,&msgh);     
    //publish the odom
    msgh.odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    msgh.set_pose = n.advertiseService("set_pose",&Message_handle::set_pose_callback,&msgh);
    msgh.path_pub = n.advertise<nav_msgs::Path>("nav_msgs/Path", 1000);

    ros::Rate r(100);
    while(ros::ok()){
        msgh.transform();
        ros::spinOnce();
        r.sleep();
    }
    return 0;

}