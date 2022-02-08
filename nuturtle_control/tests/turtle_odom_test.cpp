#include "ros/ros.h"
#include "nusim/pose.h"
#include "tf2_ros/transform_listener.h"
#include <catch_ros/catch.hpp>

TEST_CASE("test set_pose",""){
    ros::NodeHandle n;
    ros::ServiceClient setpose = n.serviceClient<nusim::pose>("set_pose");
    nusim::pose pose;
    pose.request.x = 1;
    pose.request.y = 0;
    pose.request.theta = 0;
    setpose.call(pose);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(50);
    for(int i=0; ros::ok()&& i<10;i++){
        geometry_msgs::TransformStamped transformStamped;
        try{
         transformStamped = tfBuffer.lookupTransform("odom", "red/base_footprint",
                                  ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
         ROS_WARN("%s",ex.what());
         ros::Duration(1.0).sleep();
         continue;
        }
        REQUIRE(transformStamped.transform.translation.x ==1);
        REQUIRE(transformStamped.transform.translation.y ==0);

        ros::spinOnce();
        rate.sleep();
    }

}