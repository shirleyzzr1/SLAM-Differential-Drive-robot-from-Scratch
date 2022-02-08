#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdio>
#include <cstdlib>
#include <stdio.h>
#include<vector>
#include <XmlRpcValue.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include "nusim/pose.h"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/Twist.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include <ros/console.h>
class Message_handle{
public:
    ros::Publisher sensor_pub;
    nuturtlebot_msgs::SensorData sensor;

    double max_wheel_vel;
    double last_left_encoder;
    double last_right_encoder;
    turtlelib::DiffDrive diffdrive;
    void wheel_callback(const nuturtlebot_msgs::WheelCommands& msg);

};
/// \brief the x,y,theta of the robot configuration
static double sx,sy,stheta;
// turtlelib::DiffDrive diffdrive;

/// \brief the timestep of the program
static int timestep;

/// \brief reset the timestep and reset the robot to (0,0,0)
/// request input service request message,Empty type
bool reset_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    std::vector<double> my_double_list;
    ros::param::get("/robot_start",my_double_list);
    sx = my_double_list[0];
    sy = my_double_list[1];
    stheta = my_double_list[2];
    timestep = 0;
    return true;
}
/// \brief broadcast the transform from world frame to robot red/base_footprint frame
/// x,y,theta input the pose of the robot relative to the world frame
void transform(double x,double y,double theta){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "red/base_footprint";
    //first set the translation
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = 0.0;
    //set the rotation og the robot
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
}
/// \brief draw sets of cylinders with given start point and radius
/// markerArray input the holder of the definition of markers
void drawmarker(visualization_msgs::MarkerArray &markerArray){
    //make sure that every marker has unique id;
    int id = 0;
    //the radus of the cylinders
    double radius;
    std::vector<std::vector<double>> pose;
    ros::param::get("/radius",radius);

    //using xmlrpc library to parse the 2-d vector
    XmlRpc::XmlRpcValue cylinders_start;
    ros::param::get("/cylinders_start", cylinders_start);
    //resize the cylinder vector and parse the value from param to pose
    pose.resize(cylinders_start.size());
    for(int i = 0; i < cylinders_start.size(); ++i)
    {
        for(int j = 0; j < cylinders_start[i].size(); ++j)
        {
        if(XmlRpc::XmlRpcValue::TypeDouble == cylinders_start[i][j].getType())
            pose[i].push_back(static_cast<double>(cylinders_start[i][j]));}
    }
    for(unsigned int i=0;i<pose.size();i++){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    //set the different poses
    marker.pose.position.x = pose[i][0];
    marker.pose.position.y = pose[i][1];
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    //set the scale of different directions
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = 0.25;
    //set the colors of the object
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //add the marker to the array
    markerArray.markers.push_back(marker);
    id++;
    }
    double x_length,y_length;

    ros::param::get("~x_length", x_length);
    ros::param::get("~y_length", y_length);
    std::vector<std::vector<double>> poses;
    std::vector<std::vector<double>> scales;
    std::vector<double> pose1 = {0,y_length/2};
    std::vector<double> pose2 = {-x_length/2,0};
    std::vector<double> pose3 = {0,-y_length/2};
    std::vector<double> pose4 = {x_length/2,0};

    poses.push_back(pose1);
    poses.push_back(pose2);
    poses.push_back(pose3);
    poses.push_back(pose4);

    std::vector<double> scale1 = {x_length,0.01};
    std::vector<double> scale2 = {0.01,y_length};
    std::vector<double> scale3 = {x_length,0.01};
    std::vector<double> scale4 = {0.01,y_length};

    scales.push_back(scale1);
    scales.push_back(scale2);
    scales.push_back(scale3);
    scales.push_back(scale4);
    for(unsigned int i=0;i<poses.size();i++){

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        //set the different poses
        marker.pose.position.x = poses[i][0];
        marker.pose.position.y = poses[i][1];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        //set the scale of different directions
        marker.scale.x = scales[i][0];
        marker.scale.y = scales[i][1];
        marker.scale.z = 0.25;
        //set the colors of the object
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        //add the marker to the array
        markerArray.markers.push_back(marker);
        id++;
    }
}

/// \brief this service sets the robot to move to a given place
/// req input [x:"",y:"",theta:""]
bool teleportCallback(nusim::pose::Request &req,nusim::pose::Response &res){
    sx = req.x;
    sy = req.y;
    stheta = req.theta;
    return true;
}
void Message_handle::wheel_callback(const nuturtlebot_msgs::WheelCommands& msg){
    // ROS_INFO("receive_wheel_callback");
    double motor_cmd_to_radsec;
    // ROS_WARN("%d,%d",msg.left_velocity,msg.right_velocity);
    turtlelib::Vector2D wheel_vel{msg.left_velocity*this->max_wheel_vel/256,msg.right_velocity*this->max_wheel_vel/256};
    // ROS_WARN("next line %f,%f",wheel_vel.x,wheel_vel.y);
    // ROS_INFO("receive_wheel_callback,%lf",wheel_vel.x);

    this->diffdrive.set_wheel_vel(wheel_vel);

}
int main(int argc, char ** argv){
    //init the node
    ros::init(argc, argv, "nusim");
    int rate,cylinder_num;
    double x0,y0,theta0;
    double radius,track;
    double encoder_ticks_to_rad;
    //read all the parameters from launch file
    ros::param::get("~rate",rate);
    ros::param::get("~x0",x0);
    ros::param::get("~y0",y0);
    ros::param::get("~theta0",theta0);
    ros::param::get("/wheel_radius",radius);
    ros::param::get("/track_width",track);
    ros::param::get("/encoder_ticks_to_rad",encoder_ticks_to_rad);

    //set the private namespace to nodehanle
    ros::NodeHandle n("~");
    //set the service server ~/teleport and ~reset
    ros::ServiceServer teleport = n.advertiseService("teleport",teleportCallback);
    ros::ServiceServer service = n.advertiseService("reset", reset_callback);

    //set the publiser ~/obstacles and ~timestep
    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "obstacles/", 0 ,true);
    ros::Publisher timestep_pub = n.advertise<std_msgs::UInt64>( "timestep", 0);
    std_msgs::UInt64 msg_timestep;
    timestep = ros::Time::now().toSec();
    
    //read the start point of the robot from param
    std::vector<double> my_double_list;
    ros::param::get("/robot_start",my_double_list);
    sx = my_double_list[0];
    sy = my_double_list[1];
    stheta = my_double_list[2];

    ros::Rate r(rate); // 50 hz by default

    //define the markerarray here
    visualization_msgs::MarkerArray markerArray;

    //fill all the markers 
    drawmarker(markerArray);

    //publish the markerarray
    vis_pub.publish( markerArray);

    // //publish joint_states instead of using joint_state_publisher_gui
    // ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>( "/joint_states",1);
    // sensor_msgs::JointState jointstate;
    // jointstate.name.push_back("red/wheel_left_joint");
    // jointstate.name.push_back("red/wheel_right_joint");

    // jointstate.position.push_back(0.0);
    // jointstate.position.push_back(0.0);

    Message_handle msgh;
    msgh.diffdrive.set_param(radius,track);
    msgh.last_left_encoder = 0;
    msgh.last_right_encoder = 0;
    //calculate max_wheel_vel given the max_trans_velocity is 0.22m/s
    turtlelib::Twist2D max_trans = {0,0.22,0};
    msgh.diffdrive.IK_calculate(max_trans);
    msgh.max_wheel_vel = abs(msgh.diffdrive.wheel_vel().x);
    //reset the diffdive wheel_velocity;
    msgh.diffdrive.set_wheel_vel({0,0});

    ros::Subscriber wheel_sub= n.subscribe("/wheel_cmd",1000,&Message_handle::wheel_callback,&msgh);

    msgh.sensor_pub = n.advertise<nuturtlebot_msgs::SensorData>("/sensor_data", 1000);


    while (ros::ok())
    {
        //broadcast the transformfrom world frame to robot frame
        // transform(sx,sy,stheta);
        // ROS_INFO("in ros_main function,%lf",diffdrive.wheel_vel().x);

        turtlelib::Vector2D wheel =  {msgh.diffdrive.wheel_vel().x/rate,msgh.diffdrive.wheel_vel().y/rate};
        msgh.diffdrive.wheel_pos()+=wheel;
        //publish the message on 
        msgh.sensor.stamp = ros::Time::now();
        // ROS_WARN("x: %f, y: %f"diffdrive.wheel_pos().x,diffdrive.wheel_pos().y);
        msgh.sensor.left_encoder =  msgh.diffdrive.wheel_pos().x / (2 * turtlelib::PI) * encoder_ticks_to_rad;
        msgh.sensor.right_encoder =  msgh.diffdrive.wheel_pos().y / (2 * turtlelib::PI) * encoder_ticks_to_rad;
        msgh.sensor_pub.publish(msgh.sensor);

        // publish joint states
        // jointstate.header.stamp = ros::Time::now();
        // joint_pub.publish(jointstate);
        msg_timestep.data = timestep;
        //publish the timestep
        timestep_pub.publish(msg_timestep);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}