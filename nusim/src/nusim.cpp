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
///     sensor_data(nuturtlebot_msgs::SensorData): the sensor data like encoder
///     fake_sensor(visualization_msgs::MarkerArray): the relative sensor pos in the robot basefootprint
///     laser(sensor_msgs::LaserScan): the simulated laser scan message of the robot
 ///    red_path(nav_msgs::Path):the path of the red robot
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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>
#include <iostream>
#include <math.h>       /* atan */
/// \brief pass parameters in different threads
class Message_handle{
public:
    ros::Publisher sensor_pub;
    ros::Publisher fake_sensor_pub;
    ros::Publisher laser_pub;
    ros::Publisher path_pub;

    nuturtlebot_msgs::SensorData sensor;
    double rate;
    double motor_cmd_to_radsec;

    double last_left_encoder;
    double last_right_encoder;
    turtlelib::DiffDrive diffdrive;
    turtlelib::DiffDrive reddiff;

    std::vector<double> cylinders_start_x;
    std::vector<double> cylinders_start_y;
    std::vector<geometry_msgs::PoseStamped> poses;
    
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
    double encoder_ticks_to_rad;
    int count=0;

    //mean,stddev
    void wheel_callback(const nuturtlebot_msgs::WheelCommands& msg);
    bool teleportCallback(nusim::pose::Request &req,nusim::pose::Response &res);
    void publish_sensors(const ros::TimerEvent& event);
    void main(const ros::TimerEvent& event);

    void drawmarker(visualization_msgs::MarkerArray &markerArray);
    void transform(turtlelib::Transform2D trans);

    double calculate_nearest_point(turtlelib::Vector2D robot_pos,turtlelib::Vector2D lidar_end);
    void update_pos();



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


/// \brief reset the timestep and reset the robot to (0,0,0)
/// request input service request message,Empty type
bool reset_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    std::vector<double> my_double_list;
    ros::param::get("/robot_start",my_double_list);
    sx = my_double_list[0];
    sy = my_double_list[1];
    stheta = my_double_list[2];
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
        double move_dis;
        move_dis = collision_radius+cylinder_radius-dis;
        theta = atan2(trans.translation().y-cylinders_start_y[i],trans.translation().x-cylinders_start_x[i]);
        move = {move_dis*cos(theta),move_dis*sin(theta)};
        //move the wheel, aka increase the encoder data
        // this->update_pos();
        // this->reddiff.set_body_pos({{trans.translation().x+move.x,trans.translation().x+move.x},trans.rotation()});
        //update the red robot wheel_pos
        break;
    }
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "red/base_footprint";
    //first set the translation
    transformStamped.transform.translation.x = trans.translation().x+move.x;
    transformStamped.transform.translation.y = trans.translation().y+move.y;
    
    transformStamped.transform.translation.z = 0.0;
    //set the rotation og the robot
    tf2::Quaternion q;
    q.setRPY(0, 0, trans.rotation());
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
    nav_msgs::Path path;
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = trans.translation().x+move.x;
    pose.pose.position.y = trans.translation().y+move.y;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    this->poses.push_back(pose);
    path.poses = poses;
    this->path_pub.publish(path);
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
    marker.scale.x = 2*this->cylinder_radius;
    marker.scale.y = 2*this->cylinder_radius;
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
    this->reddiff.set_wheel_vel(wheel_vel);

    // this->reddiff.FK_calculate_vel(wheel_vel);
    
}
// void calculate_distance(turtlelib::Vector2D body_pos, turtlelib::Vector2D obstacle_pos){
//     return sqrt(pow(body_pos.x-obstacle_pos.x,2)+pow(body_pos.y-obstacle_pos.y,2));
// }
double Message_handle::calculate_nearest_point(turtlelib::Vector2D robot_pos,turtlelib::Vector2D lidar_end){    
    double x1,x2,y1,y2;
    double x3,x4,y3,y4;
    double radius = this->cylinder_radius;
    double dy_sign = 1;
    double dis_min =  this->range_max-(this->resolution);
    double x_length,y_length;
    ros::param::get("x_length", x_length);
    ros::param::get("y_length", y_length);
    std::vector<double> x_point = {-0.5*x_length,0.5*x_length,0.5*x_length,-0.5*x_length};
    std::vector<double> y_point = {0.5*y_length,0.5*y_length,-0.5*y_length,-0.5*y_length};
    x1 = robot_pos.x;
    y1 = robot_pos.y;
    x2 = lidar_end.x;
    y2 = lidar_end.y;
    for(int i=0;i<4;i++){
        x3 = x_point[i];
        y3 = y_point[i];
        x4 = x_point[(i+1)%4];
        y4 = y_point[(i+1)%4];
        double D = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
        if (D==0) continue;
        double intersect_x = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/D;
        double intersect_y = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))/D;
        if((intersect_x<std::min(x1,x2)-0.001)|| (intersect_x >std::max(x1,x2)+0.001) || 
        (intersect_x<std::min(x3,x4)-0.001)|| (intersect_x >std::max(x3,x4)+0.001) ||
        (intersect_y<std::min(y1,y2)-0.001)|| (intersect_y >std::max(y1,y2)+0.001) ||
        (intersect_y<std::min(y3,y4)-0.001)|| (intersect_y >std::max(y3,y4)+0.001))continue;
        double dis1 = sqrt(pow(intersect_x-x1,2)+pow(intersect_y-y1,2));

        dis_min = std::min(dis1,dis_min);
    }
    for(int i=0;i<this->cylinders_start_x.size();i++){
        //first move the circle to the (0,0),move the x,y accordingly
        x1 = robot_pos.x-this->cylinders_start_x[i];
        y1 = robot_pos.y-this->cylinders_start_y[i];
        x2 = lidar_end.x-this->cylinders_start_x[i];
        y2 = lidar_end.y-this->cylinders_start_y[i];
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr = sqrt(dx*dx+dy*dy);
        double D = x1 * y2 - x2 * y1;
        double dy_sign = 1;

        if(dy<0)dy_sign=-1;
        double delta = radius*radius*dr*dr - D*D;
        if(delta>0){
            double intersect_x1 = (D*dy+dy_sign*dx*sqrt(delta))/(dr*dr);
            double intersect_y1 = (-D*dx+abs(dy)*sqrt(delta))/(dr*dr);

            double intersect_x2 = (D*dy-dy_sign*dx*sqrt(delta))/(dr*dr);
            double intersect_y2 = (-D*dx-abs(dy)*sqrt(delta))/(dr*dr);
            //check if the intersect is within the range of the line segment
            if(intersect_x1<std::min(x1,x2)-0.001|| (intersect_x1 >std::max(x1,x2)+0.001) || 
            intersect_x2<std::min(x1,x2)-0.001|| intersect_x2 >std::max(x1,x2)+0.001 ||
            intersect_y1<std::min(y1,y2)-0.001|| intersect_y1 >std::max(y1,y2)+0.001 ||
            intersect_y2<std::min(y1,y2)-0.001|| intersect_y2 >std::max(y1,y2)+0.001)continue;
            //choose a loser point
            double dis1 = sqrt(pow(intersect_x1-x1,2)+pow(intersect_y1-y1,2));
            double dis2 = sqrt(pow(intersect_x2-x1,2)+pow(intersect_y2-y1,2));

            if (std::min(dis1,dis2)<dis_min) dis_min = std::min(dis1,dis2);
        }
    }
    if (dis_min<(this->range_min)) dis_min = this->range_min;
    return dis_min;


}
void Message_handle::publish_sensors(const ros::TimerEvent& event){
    //publish fake sensor
        //msgh.reddiff.body_pos();
    //Twr: transformation from world to robot
    turtlelib::Transform2D Twr = this->reddiff.body_pos();
    //Tro: transformation from robot to obstacles;
    // turtlelib::Transform2D Tro;
    visualization_msgs::MarkerArray markerArray;
    std::normal_distribution<> gaussian{0,this->basic_sensor_variance};
    double max_range;
    ros::param::get("/max_range",max_range);

    for(unsigned int i=0;i<cylinders_start_x.size();i++){
        turtlelib::Vector2D cylinders_pos={this->cylinders_start_x[i],this->cylinders_start_y[i]};
        double radians = atan2(this->cylinders_start_y[i],this->cylinders_start_x[i]);
        // radians = turtlelib::normalize_angle(radians);
        turtlelib::Transform2D Two(cylinders_pos,radians);
        //robot to object translation
        turtlelib::Transform2D Tro = (Twr.inv())*Two;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "red/base_footprint";
        marker.header.stamp = ros::Time();
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        double dis = sqrt(pow(Tro.translation().x,2)+pow(Tro.translation().y,2));
        double phi = atan2(Tro.translation().y,Tro.translation().x);
        // double phi = Tro.rotation();
        dis+=gaussian(random_seed());
        phi+=gaussian(random_seed());
        if (dis>max_range) marker.action = visualization_msgs::Marker::DELETE;
        else marker.action = visualization_msgs::Marker::ADD;
        //set the different poses
        marker.pose.position.x = dis*cos(phi);
        marker.pose.position.y = dis*sin(phi);
        // ROS_INFO("pose %lf,%lf",dis*cos(phi),dis*sin(phi));
        // if(i==1)
        // ROS_INFO("dis%lf,angle:%lf,posx:%lf,posy:%lf",dis,phi,dis*cos(phi),dis*sin(phi));
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;
        //set the scale of different directions
        marker.scale.x = 2*this->cylinder_radius;
        marker.scale.y = 2*this->cylinder_radius;
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
    double angle_increment = (this->angle_max-this->angle_min)/this->samples;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "red/base_footprint";
    scan.angle_min = this->angle_min;
    scan.angle_max = this->angle_max;
    scan.angle_increment = angle_increment;
    scan.time_increment = 0.2/this->samples;
    scan.range_min = this->range_min;
    scan.range_max = this->range_max;
    std::vector<float> ranges;
    std::normal_distribution<> guassian{0,this->noise_std};
    for(int i=0;i<this->samples;i++){
        //we first calculate all the distance in world frame
        double lidar_angle = Twr.rotation() + i*angle_increment;
        turtlelib::Vector2D lidar_end = {Twr.translation().x+this->range_max*cos(lidar_angle),
                                        Twr.translation().y+this->range_max*sin(lidar_angle)};
        double dis = this->calculate_nearest_point(Twr.translation(),lidar_end);
        dis+=guassian(random_seed());
        //add noise to the lidar data
        ranges.push_back((float)dis);
    }
    scan.ranges = ranges;
    laser_pub.publish(scan);

}
void Message_handle::update_pos(){
    std::uniform_real_distribution<double> uniform{this->slip_min, this->slip_max};

    turtlelib::Vector2D wheel =  {this->diffdrive.wheel_vel().x/this->rate,this->diffdrive.wheel_vel().y/this->rate};
    //add slip to the wheel
    double eta;
    eta = uniform(random_seed());
    wheel += wheel*(eta/10);
    this->diffdrive.wheel_pos()+=wheel;

    //publish the message on 
    this->sensor.stamp = ros::Time::now();
    // ROS_WARN("x: %f, y: %f"diffdrive.wheel_pos().x,diffdrive.wheel_pos().y);
    this->sensor.left_encoder =  this->diffdrive.wheel_pos().x / (2 * turtlelib::PI) * encoder_ticks_to_rad;
    this->sensor.right_encoder =  this->diffdrive.wheel_pos().y / (2 * turtlelib::PI) * encoder_ticks_to_rad;
    this->sensor_pub.publish(this->sensor);


    turtlelib::Vector2D red_wheel =  {this->reddiff.wheel_vel().x/this->rate,this->reddiff.wheel_vel().y/this->rate};
    turtlelib::Vector2D new_red_wheel = this->reddiff.wheel_pos()+red_wheel;
    // ROS_INFO("blue vel %lf,%lf",this->diffdrive.wheel_vel().x,this->diffdrive.wheel_vel().y);
    // ROS_INFO("blue pos %lf,%lf",this->diffdrive.wheel_pos().x,this->diffdrive.wheel_pos().y);
    // ROS_INFO("red vel %lf,%lf",this->reddiff.wheel_vel().x,this->reddiff.wheel_vel().y);
    // ROS_INFO("red pos %lf,%lf",this->reddiff.wheel_pos().x,this->reddiff.wheel_pos().y);
    this->reddiff.FK_calculate(new_red_wheel);
    turtlelib::Transform2D trans = {{this->reddiff.body_pos().translation().x,this->reddiff.body_pos().translation().y},\
                turtlelib::normalize_angle(this->reddiff.body_pos().rotation())};
    this->reddiff.set_body_pos(trans);
}
void Message_handle::main(const ros::TimerEvent& event){
    this->update_pos();
    this->transform(this->reddiff.body_pos());

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

    ros::param::get("/rate_nusim",rate);
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

    msgh.laser_pub = n.advertise<sensor_msgs::LaserScan>("/laser", 1000);
    msgh.path_pub = n.advertise<nav_msgs::Path>("/red_path", 1000);

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
    msgh.encoder_ticks_to_rad =encoder_ticks_to_rad;
    msgh.count=0;

    //the timer to publish the sensor and laser data every 0.2 seconds
    ros::Timer timer_publish = n.createTimer(ros::Duration(0.2), &Message_handle::publish_sensors,&msgh);

    // ros::Timer timer_main = n.createTimer(ros::Duration(0.02), &Message_handle::main,&msgh);
    // ros::spin();
    ros::Rate r(msgh.rate);

    while (ros::ok())
    {
        //broadcast the transformfrom world frame to robot frame
        msgh.update_pos();
        msgh.transform(msgh.reddiff.body_pos());
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}