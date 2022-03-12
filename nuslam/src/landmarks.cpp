#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
class Message_handle{
    lidar_callback(sensor_msgs::LaserScan msg);
}
Message_handle::lidar_callback(sensor_msgs::LaserScan msg){
    
}
int main(int argc, char ** argv){
    ros::init(argc, argv, "landmark");
    msgh = Message_handle();
    ros::Subscriber lidar_sub= n.subscribe("/laser",1000,&Message_handle::lidar_callback,&msgh);

}