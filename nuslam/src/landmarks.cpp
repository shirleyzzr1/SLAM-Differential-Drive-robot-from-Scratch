#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "turtlelib/circle_detect.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
class Message_handle{
public:
    turtlelib::Circle circle;
    ros::Publisher vis_pub;
    void lidar_callback(sensor_msgs::LaserScan msg);
    double dist_thresh;
    double std_angle;
    double min_mean_angle;
    double max_mean_angle;
    double radius_range;
    std::string marker_frame_id;

};
void Message_handle::lidar_callback(sensor_msgs::LaserScan msg){
    // ROS_INFO("get laser data");
    circle.clear();
    // ROS_INFO("circle_clear_success");
    circle.range_cluster(msg.ranges,this->dist_thresh);
    // ROS_INFO("range_cluster");
    circle.classification(this->std_angle,this->min_mean_angle,this->max_mean_angle);
    // ROS_INFO("classification");
    circle.circle_fitting(this->radius_range);
    // ROS_INFO("circle_fitting"); 
    // ROS_INFO("circle_nums:%d",this->circle.centers.size());
    visualization_msgs::MarkerArray markerArray;
    // ROS_INFO("x:%lf,y:%lf",circle.centers[i].x,circle.centers[i].y);
    // ROS_INFO("circle numbers:%d",circle.centers.size());
    
    for(unsigned int i=0;i<this->circle.centers.size();i++){
    //the read marker pose
        // ROS_INFO("circle_centers x:%lf,y:%lf",this->circle.centers[i].x,this->circle.centers[i].y);
        visualization_msgs::Marker marker;
        marker.header.frame_id = this->marker_frame_id;
        marker.header.stamp = ros::Time();
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        //set the different poses
        marker.pose.position.x = circle.centers[i].x;
        marker.pose.position.y = circle.centers[i].y;
        marker.pose.orientation.w = 1.0;
        //set the scale of different directions
        marker.scale.x = 2*circle.radius[i];
        marker.scale.y = 2*circle.radius[i];
        marker.scale.z = 0.25;
        //set the colors of the object
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        //add the marker to the array
        markerArray.markers.push_back(marker);
    }
    vis_pub.publish(markerArray);

}
int main(int argc, char ** argv){
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle n;
    double dist_thresh;
    double std_angle;
    double min_mean_angle;
    double max_mean_angle;
    double radius_range;
    
    Message_handle msgh;
    ros::param::get("/dist_thresh",dist_thresh);
    ros::param::get("/std_angle",std_angle);
    ros::param::get("/min_mean_angle",min_mean_angle);
    ros::param::get("/max_mean_angle",max_mean_angle);
    ros::param::get("/radius_range",radius_range);
    n.param<std::string>("/marker_frame_id", msgh.marker_frame_id, "red/base_footprint");

    msgh.dist_thresh = dist_thresh;
    msgh.std_angle = std_angle;
    msgh.min_mean_angle = min_mean_angle;
    msgh.max_mean_angle = max_mean_angle;
    msgh.radius_range = radius_range;
    ros::Subscriber lidar_sub= n.subscribe("/laser",1000,&Message_handle::lidar_callback,&msgh);
    msgh.vis_pub = n.advertise<visualization_msgs::MarkerArray>( "/clustering_circle", 0 ,true);
    ros::Rate r(50);

    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}