#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include <catch_ros/catch.hpp>
#include "sensor_msgs/JointState.h"
void wheel_sub_callback(const nuturtlebot_msgs::WheelCommands& msg){
    REQUIRE(msg.left_velocity==256);
    REQUIRE(msg.right_velocity==256);
}
//set robot to none when running this test
TEST_CASE("pure translation",""){
    ros::NodeHandle n;
    ros::Subscriber wheel_sub = n.subscribe("wheel_cmd",1000,wheel_sub_callback);
    ros::Publisher cmd_pub =  n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    geometry_msgs::Twist tw;
    tw.linear.x = 0.22;
    ros::Rate r(50);
    for(int i=0; ros::ok()&& i<10;i++){
        cmd_pub.publish(tw);
        ros::spinOnce();
        r.sleep();
    }
}
void wheel_rotation_callback(const nuturtlebot_msgs::WheelCommands& msg){
    REQUIRE(msg.left_velocity==-256);
    REQUIRE(msg.right_velocity==256);
}
TEST_CASE("pure rotation",""){
    ros::NodeHandle n;
    ros::Subscriber wheel_sub = n.subscribe("wheel_cmd",1000,wheel_rotation_callback);
    ros::Publisher cmd_pub =  n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    geometry_msgs::Twist tw;
    tw.angular.z = 2.84;
    ros::Rate r(50);
    for(int i=0; ros::ok()&& i<50;i++){
        cmd_pub.publish(tw);
        ros::spinOnce();
        r.sleep();
    }
}

void joint_state_callback(const sensor_msgs::JointState& msg){
    // REQUIRE(msg.velocity[0]==0);
    // REQUIRE(msg.velocity[1]==256);
    REQUIRE(msg.position[0]==Approx(1.53).epsilon(0.01));
    REQUIRE(msg.position[1]==Approx(1.53).epsilon(0.01));
}
sensor_msgs::JointState joint_state;
TEST_CASE("encoder data convertion",""){
    ros::NodeHandle n;
    
    ros::Subscriber joint_state_sub = n.subscribe("joint_states",1000,joint_state_callback);
    ros::Publisher sensor_pub =  n.advertise<nuturtlebot_msgs::SensorData>("sensor_data", 1000);

    nuturtlebot_msgs::SensorData sensor;
    sensor.left_encoder = 1000;
    sensor.right_encoder = 1000;
    sensor_pub.publish(sensor);
    ros::Rate r(50);

    for(int i=0; ros::ok()&& i<50;i++){
        sensor_pub.publish(sensor);
        ros::spinOnce();
        r.sleep();
    }
}