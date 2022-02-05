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


static tf2_ros::TransformBroadcaster br;

class Message_handle{
public:
    std::string body_id,odom_id,wheel_left,wheel_right; 
    ros::Publisher odom_pub;
    ros::ServiceServer set_pose;

    turtlelib::DiffDrive diffdrive;

    void transform();
    bool set_pose_callback(nusim::pose::Request &req,nusim::pose::Response &res);
    void joint_state_callback(const sensor_msgs::JointState& msg);


};

/// \brief broadcast the transform from world frame to robot red/base_footprint frame
/// x,y,theta input the pose of the robot relative to the world frame
void Message_handle::transform(){
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = this->odom_id ;
    transformStamped.child_frame_id = this->body_id ;

    //first set the translation
    transformStamped.transform.translation.x = this->diffdrive.body_pos().x;
    transformStamped.transform.translation.y = this->diffdrive.body_pos().y;
    transformStamped.transform.translation.z = 0.0;
    //set the rotation og the robot
    tf2::Quaternion q;
    q.setRPY(0, 0, this->diffdrive.body_pos().theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
}
void Message_handle::joint_state_callback(const sensor_msgs::JointState& msg){
    nav_msgs::Odometry odom;
    odom.header.frame_id = this->odom_id;
    odom.header.stamp = ros::Time::now();
    odom.child_frame_id = this->body_id;
    turtlelib::Vector2D wheel_vel{msg.velocity[0],msg.velocity[1]};
    turtlelib::Vector2D wheel_pos{msg.position[0],msg.position[1]};
    this->diffdrive.FK_calculate(wheel_pos,wheel_vel);
    
    geometry_msgs::Pose pos;
    pos.position.x = this->diffdrive.body_pos().x;
    pos.position.y = this->diffdrive.body_pos().y;
    pos.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, this->diffdrive.body_pos().theta);
    pos.orientation.x = q.x();
    pos.orientation.y = q.y();
    pos.orientation.z = q.z();
    pos.orientation.w = q.w();
    odom.pose.pose = pos;

    geometry_msgs::Twist twist;
    odom.twist.twist.linear.x = this->diffdrive.bodytwist().xdot;
    odom.twist.twist.linear.y = this->diffdrive.bodytwist().ydot;
    odom.twist.twist.linear.z = 0;

    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = this->diffdrive.bodytwist().thetadot;



    odom_pub.publish(odom);


}
bool Message_handle::set_pose_callback(nusim::pose::Request &req,nusim::pose::Response &res){
    turtlelib::Configure conf{req.theta,req.x,req.y};
    diffdrive.set_body_pos(conf);
    return true;
}
int main(int argc, char ** argv){
    //init the node here
    ros::init(argc, argv, "odometry");

    double radius,track;
    ros::param::get("radius",radius);
    ros::param::get("track",track);

    Message_handle msgh;
    msgh.diffdrive.set_param(radius,track);
    ros::NodeHandle n;
    if (!n.getParam("body_id", msgh.body_id))
    {
        ROS_DEBUG_STREAM("not initialize the body id"); 
        ros::shutdown();
    }
    if (!n.getParam("left_whee_joint", msgh.wheel_left))
    {
        ROS_DEBUG_STREAM("not specify the left_wheel_joint"); 
        ros::shutdown();
    }
    if (!n.getParam("right_whee_joint", msgh.wheel_right))
    {
        ROS_DEBUG_STREAM("not specify the right_wheel_joint"); 
        ros::shutdown();
    }
    n.param<std::string>("odom_id", msgh.odom_id, "odom");

    //subscribe to the cmd_vel
    ros::Subscriber joint_sub = n.subscribe("joint_states",1000,
        &Message_handle::joint_state_callback,&msgh);

    msgh.odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);

    msgh.set_pose = n.advertiseService("set_pose",&Message_handle::set_pose_callback,&msgh);

    while(ros::ok()){
        msgh.transform();
    }

}