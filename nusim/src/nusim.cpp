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
#include "nusim/pose.h"

/// \brief the x,y,theta of the robot configuration
static double sx,sy,stheta;

/// \brief the timestep of the program
static int timestep;

/// \brief reset the timestep and reset the robot to (0,0,0)
/// request input service request message,Empty type
bool reset_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    sx = 0;
    sy = 0;
    stheta = 0;
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
            pose[i].push_back(static_cast<double>(cylinders_start[i][j]));
        }
    }
    //set the properties of different markers
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
}
/// \brief this service sets the robot to move to a given place
/// req input [x:"",y:"",theta:""]
bool teleportCallback(nusim::pose::Request &req,nusim::pose::Response &res){
    sx = req.x;
    sy = req.y;
    stheta = req.theta;
    return true;
}
int main(int argc, char ** argv){
    //init the node
    ros::init(argc, argv, "nusim");
    int rate,cylinder_num;
    double x0,y0,theta0;
    //read all the parameters from launch file
    ros::param::get("~rate",rate);
    ros::param::get("~x0",x0);
    ros::param::get("~y0",y0);
    ros::param::get("~theta0",theta0);
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

    while (ros::ok())
    {
        //broadcast the transformfrom world frame to robot frame
        transform(sx,sy,stheta);
        //fill all the markers 
        drawmarker(markerArray);
        //publish the markerarray
        vis_pub.publish( markerArray);
        msg_timestep.data = timestep;
        //publish the timestep
        timestep_pub.publish(msg_timestep);
        ros::spinOnce();
        //ros::spin();
        r.sleep();
    }
    return 0;
}