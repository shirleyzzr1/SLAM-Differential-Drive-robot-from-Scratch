/// \file nusim.cpp
/// \brief simulate the robot world by providing the sensor data
/// PARAMSTERS:
///     x0:(double):the start point of the robot in x axis
///     y0(double): the start point of the robot in y axis
///     theta0(double): the start ponit of the robot direction
///     rate(double): the rate of the main loop
///     x_length(double): the length of the wall in x axis
///     y_length(double): the length of the wall in y axis
/// SUBSCRIBERS:
///    wheel_cmd(nuturtlebot_msgs::WheelCommands): the velocity of the wheels
/// PUBLISHERS:
///    sensor_data(nuturtlebot_msgs::SensorData): the sensor data like encoder
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
#include <sensor_msgs/LaserScan.h>
#include <random>
#include <iostream>
#include <math.h>       /* atan */

/// \brief pass parameters in different threads
class Message_handle{
public:
    ros::Publisher sensor_pub;
    ros::Publisher fake_sensor_pub;

    ros::Publisher laser_pub;
    nuturtlebot_msgs::SensorData sensor;
    double rate;
    double motor_cmd_to_radsec;

    double last_left_encoder;
    double last_right_encoder;
    turtlelib::DiffDrive diffdrive;
    turtlelib::DiffDrive reddiff;

    std::vector<double> cylinders_start_x;
    std::vector<double> cylinders_start_y;
    
    double cylinder_radius;
    double wheel_mean;
    double wheel_stddev;
    double slip_min;
    double slip_max;
    //lidar parameter
    double angle_min;
    double angle_max;
    double samples;
    double range_min;
    double range_max;
    double resolution;
    double noise_std;
    double basic_sensor_variance;

    //mean,stddev
    void wheel_callback(const nuturtlebot_msgs::WheelCommands& msg);
    bool teleportCallback(nusim::pose::Request &req,nusim::pose::Response &res);
    void publish_sensors(const ros::TimerEvent& event);

    void drawmarker(visualization_msgs::MarkerArray &markerArray);
    void transform(turtlelib::Transform2D trans);


};
std::mt19937& random_seed(){
    //randomness for initializing random seed
    static std::random_device rd{}; 

    // Mersenne twister PRNG, initialized with seed 
    //from previous random device instance
    static std::mt19937 gen{rd()}; 
    return gen;
}

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
void Message_handle::transform(turtlelib::Transform2D trans){
    //check for collision after each update
    double collision_radius;
    double theta,dis;
    ros::param::get("/collision_radius",collision_radius);
    turtlelib::Vector2D move = {0,0};
    //if intersect with other cylinder, move towards the tangent line
    for(int i=0;i<cylinders_start_x.size();i++){
        dis = sqrt(pow(trans.translation().x-cylinders_start_x[i],2)+pow(trans.translation().y-cylinders_start_y[i],2));
        if (dis>=(collision_radius+cylinder_radius))continue;
        // ROS_INFO("collision happened");
        double move_dis = collision_radius+cylinder_radius-dis;
        theta = atan2(trans.translation().y-cylinders_start_y[i],trans.translation().x-cylinders_start_x[i]);
        move = {move_dis*cos(theta),move_dis*sin(theta)};
        //move the wheel
        turtlelib::Twist2D twist = {0,move.x,move.y};
        this->diffdrive.IK_calculate(twist);
        this->diffdrive.set_wheel_pos(this->diffdrive.wheel_pos()+this->diffdrive.wheel_vel()*((double)1/rate));

        this->reddiff.IK_calculate(twist);
        this->reddiff.FK_calculate_vel(this->reddiff.wheel_vel()*((double)1/rate));
        break;
    }
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "red/base_footprint";
    //first set the translation
    transformStamped.transform.translation.x = this->reddiff.body_pos().translation().x;
    transformStamped.transform.translation.y = this->reddiff.body_pos().translation().y;
    transformStamped.transform.translation.z = 0.0;
    //set the rotation og the robot
    tf2::Quaternion q;
    q.setRPY(0, 0, trans.rotation());
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
}
/// \brief draw sets of cylinders with given start point and radius
/// markerArray input the holder of the definition of markers
void Message_handle::drawmarker(visualization_msgs::MarkerArray &markerArray){
    //make sure that every marker has unique id;
    int id = 0;

    for(unsigned int i=0;i<this->cylinders_start_x.size();i++){
    //the read marker pose
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    //set the different poses
    marker.pose.position.x = this->cylinders_start_x[i];
    marker.pose.position.y = this->cylinders_start_y[i];
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    //set the scale of different directions
    marker.scale.x = this->cylinder_radius;
    marker.scale.y = this->cylinder_radius;
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

    //generate the wall
    double x_length,y_length;
    ros::param::get("x_length", x_length);
    ros::param::get("y_length", y_length);
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
bool Message_handle::teleportCallback(nusim::pose::Request &req,nusim::pose::Response &res){

    this->reddiff.set_body_pos({{req.x,req.y},req.theta});
    return true;
}
void Message_handle::wheel_callback(const nuturtlebot_msgs::WheelCommands& msg){
    // ROS_INFO("receive_wheel_callback");
    // ROS_WARN("%d,%d",msg.left_velocity,msg.right_velocity);
    turtlelib::Vector2D wheel_vel{(double)msg.left_velocity*this->motor_cmd_to_radsec,\
    (double)msg.right_velocity*this->motor_cmd_to_radsec};
    double sample;
    //add gaussian noise to the angular velocity,no noise if angular velocities are zero
    std::normal_distribution<> guassian{wheel_mean,wheel_stddev};

    if(abs(wheel_vel.x) >= 0.1 ||  abs(wheel_vel.y) >= 0.1){
        // instance of class std::normal_distribution with specific mean and stddev
        sample = guassian(random_seed());
        wheel_vel.x += sample;
        wheel_vel.y += sample;
    }

    turtlelib::DiffDrive diff;
    this->diffdrive.set_wheel_vel(wheel_vel);
    // this->diffdrive.FK_calculate(wheel_vel);
    this->reddiff.FK_calculate_vel(wheel_vel*((double)1/rate));
    
}
// void calculate_distance(turtlelib::Vector2D body_pos, turtlelib::Vector2D obstacle_pos){
//     return sqrt(pow(body_pos.x-obstacle_pos.x,2)+pow(body_pos.y-obstacle_pos.y,2));
// }
void Message_handle::publish_sensors(const ros::TimerEvent& event){
    //publish fake sensor
        //msgh.reddiff.body_pos();
    turtlelib::Transform2D Twr = this->reddiff.body_pos();
    turtlelib::Transform2D Tro;
    visualization_msgs::MarkerArray markerArray;
    std::normal_distribution<> gaussian{0,this->basic_sensor_variance};
    double max_range;
    ros::param::get("/max_range",max_range);

    for(unsigned int i=0;i<cylinders_start_x.size();i++){
        turtlelib::Vector2D trans={this->cylinders_start_x[i],this->cylinders_start_y[i]};
        turtlelib::Transform2D Two = turtlelib::Transform2D(trans);
        Tro = (Twr.inv())*Two;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "red/base_footprint";
        marker.header.stamp = ros::Time();
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        if (Tro.rotation()>max_range) marker.action = visualization_msgs::Marker::DELETE;
        else marker.action = visualization_msgs::Marker::ADD;
        //set the different poses
        marker.pose.position.x = Tro.translation().x+gaussian(random_seed());
        marker.pose.position.y = Tro.translation().y+gaussian(random_seed());
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;
        //set the scale of different directions
        marker.scale.x = this->cylinder_radius;
        marker.scale.y = this->cylinder_radius;
        marker.scale.z = 0.25;
        //set the colors of the object
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //add the marker to the array
        markerArray.markers.push_back(marker);
    }
    fake_sensor_pub.publish(markerArray);
    //publish laser data
    sensor_msgs::LaserScan scan;
    scan.angle_min = this->angle_min;
    scan.angle_max = this->angle_max;
    scan.angle_increment = (this->angle_max-this->angle_min)/this->samples;
    scan.time_increment = 0.2/this->samples;
    scan.range_min = this->range_min;
    scan.range_max = this->range_max;
    // laser_pub.publihs(scan);

}
int main(int argc, char ** argv){
    //init the node
    ros::init(argc, argv, "nusim");
    int rate,cylinder_num;
    std::vector<double> cylinders_start_x,cylinders_start_y;
    double cylinder_radius;
    double x0,y0,theta0;
    double radius,track;
    double encoder_ticks_to_rad,motor_cmd_to_radsec;
    double wheel_mean,wheel_stddev;
    double range_min,range_max,resolution,angle_min,angle_max,samples,noise_std;
    double slip_min,slip_max;
    double basic_sensor_variance;
    //read all the parameters from launch file
    ros::param::get("/cylinders_start_x",cylinders_start_x);
    ros::param::get("/cylinders_start_y",cylinders_start_y);
    ros::param::get("/cylinder_radius",cylinder_radius);

    ros::param::get("/rate",rate);
    ros::param::get("/x0",x0);
    ros::param::get("/y0",y0);
    ros::param::get("/theta0",theta0);
    ros::param::get("/wheel_radius",radius);
    ros::param::get("/track_width",track);
    ros::param::get("/encoder_ticks_to_rad",encoder_ticks_to_rad);
    ros::param::get("/motor_cmd_to_radsec",motor_cmd_to_radsec);
    ros::param::get("/wheel_mean",wheel_mean);
    ros::param::get("/wheel_stddev",wheel_stddev);
    //slip parameters
    ros::param::get("/slip_min",slip_min);
    ros::param::get("/slip_max",slip_max);
    //lidar parameters
    ros::param::get("/range_min",range_min);
    ros::param::get("/range_max",range_max);
    ros::param::get("/resolution",resolution);
    ros::param::get("/angle_min",angle_min);
    ros::param::get("/angle_max",angle_max);
    ros::param::get("/samples",samples);
    ros::param::get("/noise_std",noise_std);

    ros::param::get("/basic_sensor_variance",basic_sensor_variance);
    Message_handle msgh;

    //set the private namespace to nodehanle
    ros::NodeHandle n("~");
    //set the service server ~/teleport and ~reset
    ros::ServiceServer service = n.advertiseService("reset", reset_callback);

    //set the publiser ~/obstacles and ~timestep
    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "obstacles", 0 ,true);
    ros::Publisher timestep_pub = n.advertise<std_msgs::UInt64>( "timestep", 0);
    std_msgs::UInt64 msg_timestep;
    timestep = ros::Time::now().toSec();

    ros::Rate r(rate); // 50 hz by default

    //define the markerarray here
    visualization_msgs::MarkerArray markerArray;
    msgh.cylinders_start_x = cylinders_start_x;
    msgh.cylinders_start_y = cylinders_start_y;
    msgh.cylinder_radius = cylinder_radius;
    //fill all the markers 
    msgh.drawmarker(markerArray);

    //publish the markerarray
    vis_pub.publish(markerArray);

    msgh.diffdrive.set_param(radius,track);
    msgh.last_left_encoder = 0;
    msgh.last_right_encoder = 0;
    msgh.motor_cmd_to_radsec = motor_cmd_to_radsec;
    //reset the diffdive wheel_velocity;
    // msgh.diffdrive.set_wheel_vel({0,0});
    msgh.reddiff.set_param(radius,track);
    turtlelib::Vector2D initial_pos = {x0,y0};
    turtlelib::Transform2D trans(initial_pos,theta0);
    msgh.reddiff.set_body_pos(trans);
    
    msgh.rate = rate;
    ros::ServiceServer teleport = n.advertiseService("teleport",&Message_handle::teleportCallback,&msgh);

    ros::Subscriber wheel_sub= n.subscribe("/wheel_cmd",1000,&Message_handle::wheel_callback,&msgh);


    msgh.sensor_pub = n.advertise<nuturtlebot_msgs::SensorData>("/sensor_data", 1000);
    msgh.fake_sensor_pub = n.advertise<visualization_msgs::MarkerArray>( "/fake_sensor", 0);

    msgh.laser_pub = n.advertise<sensor_msgs::LaserScan>("sensor_msgs/LaserScan", 1000);

    msgh.wheel_mean = wheel_mean;
    msgh.wheel_stddev = wheel_stddev;
    msgh.slip_min = slip_min;
    msgh.slip_max = slip_max;

    msgh.range_min = range_min;
    msgh.range_max = range_max;
    msgh.resolution = resolution;
    msgh.angle_min = angle_min;
    msgh.angle_max = angle_max;
    msgh.samples = samples;
    msgh.noise_std = noise_std;
    msgh.basic_sensor_variance = basic_sensor_variance;

    //the timer to publish the sensor and laser data every 0.2 seconds
    ros::Timer timer_sensors = n.createTimer(ros::Duration(0.2), &Message_handle::publish_sensors,&msgh);

    std::uniform_real_distribution<double> uniform{msgh.slip_min, msgh.slip_max};

    while (ros::ok())
    {
        //broadcast the transformfrom world frame to robot frame
        msgh.transform(msgh.reddiff.body_pos());
        // ROS_INFO("in ros_main function,%lf",diffdrive.wheel_vel().x);

        turtlelib::Vector2D wheel =  {msgh.diffdrive.wheel_vel().x/msgh.rate,msgh.diffdrive.wheel_vel().y/msgh.rate};
        //add slip to the wheel
        double eta;
        eta = uniform(random_seed());

        wheel += wheel*(eta/10);
        msgh.diffdrive.wheel_pos()+=wheel;

        //publish the message on 
        msgh.sensor.stamp = ros::Time::now();
        // ROS_WARN("x: %f, y: %f"diffdrive.wheel_pos().x,diffdrive.wheel_pos().y);
        msgh.sensor.left_encoder =  msgh.diffdrive.wheel_pos().x / (2 * turtlelib::PI) * encoder_ticks_to_rad;
        msgh.sensor.right_encoder =  msgh.diffdrive.wheel_pos().y / (2 * turtlelib::PI) * encoder_ticks_to_rad;
        msgh.sensor_pub.publish(msgh.sensor);

        msg_timestep.data = timestep;
        //publish the timestep
        timestep_pub.publish(msg_timestep);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}