#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
class Message_handle{
public:
    void lidar_callback(sensor_msgs::LaserScan msg);
};
void Message_handle::lidar_callback(sensor_msgs::LaserScan msg){
    
}
int main(int argc, char ** argv){
    ros::init(argc, argv, "landmark");
    ros::NodeHandle n;
    Message_handle msgh;
    ros::Subscriber lidar_sub= n.subscribe("/laser",1000,&Message_handle::lidar_callback,&msgh);

}