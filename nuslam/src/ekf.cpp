/// \file ekf.cpp
/// \brief simulate the robot in the world frame and apply extended kalman filter to the robot
/// PARAMSTERS:
///     cylinders_num: the number of cylinders obstacle
///     std_q: the std of the process noise of the robot model
///     std_r: the std of the noise in measurement model
///     wheel_radius: the radius of the robot wheel
///     track_width: the width of the robot track
///     cylinder_radius: the radius of the cylinder
/// SUBSCRIBERS:
///     fake_sensor(visualization_msgs::MarkerArray): the fake sensor data(the real measurments in the ekf)
///     /blue/joint_states(sensor_msgs::JointState): the joint state message for the blue robot
/// PUBLISHERS:
///    green/nav_msgs/Path(nav_msgs::Path): the path for the green robot
///    blue/nav_msgs/Path(nav_msgs::Path): the path for the blue robot
///    odom(nav_msgs::Odometry): the odometry for the robot
///    slam_obstacles(visualization_msgs::MarkerArray): the estimated obstacle position

#include <armadillo>
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "sensor_msgs/JointState.h"
#include "turtlelib/diff_drive.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>    

class Message_handle{
public:
    std::vector<int> initialize_landmark;
    int cylinders_num;
    int max_num;
    double rate;
    double new_landmark_thresh;
    arma::vec3 accumulated_twist;
    arma::mat current_state;
    arma::mat covariance;
    arma::mat H;
    arma::mat K;
    arma::mat real_measurements;
    arma::mat theoretical_measurements;
    double first_joint_flag;
    double cylinder_radius;

    std::vector<double> std_q;
    std::vector<double> std_r;

    std::vector<geometry_msgs::PoseStamped> green_poses;
    std::vector<geometry_msgs::PoseStamped> blue_poses;

    ros::Publisher green_path_pub;
    ros::Publisher blue_path_pub;

    turtlelib::DiffDrive diffdrive;

    ros::Publisher odom_pub;
    ros::Publisher vis_pub;

    void sensor_callback(const visualization_msgs::MarkerArray& msg);
    void lidar_callback(const visualization_msgs::MarkerArray& msg);

    void data_association(const visualization_msgs::MarkerArray& markers);
    void ekf_update();
    void ekf_predict();
    void transform();
    void transform_green();
    void map_odom_tf();

    void joint_state_callback(const sensor_msgs::JointState& msg);
    void drawmarker(visualization_msgs::MarkerArray &markerArray);

};

///\brief get the fake sensor data information
/// \param msg: the markerarray message
void Message_handle::sensor_callback(const visualization_msgs::MarkerArray& msg){
    this->cylinders_num = 3;
    this->ekf_predict();
    for(int i=0;i<this->cylinders_num;i++){
        double relative_x = msg.markers[i].pose.position.x;
        double relative_y = msg.markers[i].pose.position.y;
        //relative distance, relative theta
        arma::vec2 z={sqrt(relative_x*relative_x+relative_y*relative_y),turtlelib::normalize_angle(atan2(relative_y,relative_x))};
        if(this->initialize_landmark[i]){
            this->current_state(3+2*i,0) = this->current_state(1,0)+z[0]*cos(z[1]+this->current_state(0,0));
            this->current_state(3+2*i+1,0) = this->current_state(2,0)+z[0]*sin(z[1]+this->current_state(0,0));
            this->initialize_landmark[i] = 0;
        }
        (this->real_measurements)(2*i,0) = z[0];
        (this->real_measurements)(2*i+1,0) = z[1];

        //caluclate the theoretical_measurements;
        //mx - x_state;my-y_state
        relative_x =this->current_state(3+2*i,0)-this->current_state(1,0);
        relative_y =this->current_state(4+2*i,0)-this->current_state(2,0);
        double d= relative_x*relative_x+relative_y*relative_y;
        arma::vec2 z_theo={sqrt(d),turtlelib::normalize_angle(atan2(relative_y,relative_x))};
        (this->theoretical_measurements)(2*i,0) = z_theo[0];
        (this->theoretical_measurements)(2*i+1,0) = turtlelib::normalize_angle(z_theo[1]-this->current_state(0,0));

        //the derivate with respect to the state
        arma::mat Hj = arma::zeros<arma::mat>(2,3+2*this->cylinders_num);
        Hj(arma::span(0,1), arma::span(0,2))=arma::mat({{0,-relative_x/sqrt(d),-relative_y/sqrt(d)},{-1,relative_y/d,-relative_x/d}});
        Hj(arma::span(0,1), arma::span(2*(i+1)+1,2*(i+1)+2))=arma::mat({{relative_x/sqrt(d),relative_y/sqrt(d)},{-relative_y/d,relative_x/d}});
        this->H.rows(2*i,2*i+1) = Hj;
    }
    this->ekf_update();
    //define the markerarray here
    visualization_msgs::MarkerArray markerArray;
    this->drawmarker(markerArray);
    this->vis_pub.publish(markerArray);

    //publish the green robot pose in the map frame
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = this->current_state(1,0);
    pose.pose.position.y = this->current_state(2,0);
    this->green_poses.push_back(pose);
    path.poses = this->green_poses;
    this->green_path_pub.publish(path);

    // this->transform_green();

}
//use the landmarks detected circle to update the ekf-slam
void Message_handle::lidar_callback(const visualization_msgs::MarkerArray& msg){
    this->ekf_predict();
    //deal with data association
    arma::mat R = this->std_r[0] * arma::eye<arma::mat>(2,2);
    int n = this->cylinders_num;
    ROS_INFO("before%d",this->cylinders_num);
    // arma::vec lidar_map_landmark(msg.markers.size());
    arma::vec landmark_mindis(this->cylinders_num);
    landmark_mindis.fill(1000);
    arma::vec landmark_lidar(max_num);
    landmark_lidar.fill(-1);
    ROS_INFO("current_state1:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",\
    this->current_state(0,0),this->current_state(1,0),this->current_state(2,0),\
    this->current_state(3,0),this->current_state(4,0),this->current_state(5,0),\
    this->current_state(6,0),this->current_state(7,0),this->current_state(8,0),\
    this->current_state(9,0),this->current_state(10,0),this->current_state(11,0),\
    this->current_state(12,0),this->current_state(13,0));
    for(int i=0;i<msg.markers.size();i++){
        double relative_x,relative_y;
        relative_x = msg.markers[i].pose.position.x;
        relative_y = msg.markers[i].pose.position.y;
        arma::vec2 lidar_z={sqrt(relative_x*relative_x+relative_y*relative_y),turtlelib::normalize_angle(atan2(relative_y,relative_x))};
        double lidar_real_x = this->current_state(1,0)+lidar_z[0]*cos(lidar_z[1]+this->current_state(0,0));
        double lidar_real_y = this->current_state(2,0)+lidar_z[0]*sin(lidar_z[1]+this->current_state(0,0));
        ROS_INFO("mark realx %lf",lidar_real_x);
        ROS_INFO("mark realy %lf",lidar_real_y);
        if(n==0){
            this->cylinders_num+=1;
            landmark_lidar[i] = i;
        }
        else
        {
            arma::vec distance = arma::vec(n);
            for(int j=0;j<n;j++){
                //mx - x_state;my-y_state
                double relative_x =this->current_state(3+2*j,0)-this->current_state(1,0);
                double relative_y =this->current_state(4+2*j,0)-this->current_state(2,0);
                double d= relative_x*relative_x+relative_y*relative_y;
                arma::vec2 z_theo={sqrt(d),turtlelib::normalize_angle(atan2(relative_y,relative_x)-this->current_state(0,0))};

                arma::mat h = arma::zeros<arma::mat>(2,3+2*n);
                h(arma::span(0,1), arma::span(0,2))=arma::mat({{0,-relative_x/sqrt(d),-relative_y/sqrt(d)},{-1,relative_y/d,-relative_x/d}});
                h(arma::span(0,1), arma::span(2*(j+1)+1,2*(j+1)+2))=arma::mat({{relative_x/sqrt(d),relative_y/sqrt(d)},{-relative_y/d,relative_x/d}});

                //2*2
                arma::mat dis_cov = h*this->covariance(arma::span(0,2+2*n),arma::span(0,2+2*n))*(h.t())+R;
                double real_x = msg.markers[i].pose.position.x;
                double real_y = msg.markers[i].pose.position.y;
                //relative distance, relative theta
                arma::vec2 z_real={sqrt(real_x*real_x+real_y*real_y),turtlelib::normalize_angle(atan2(real_y,real_x))};
                arma::mat dif = arma::colvec({z_real[0]-z_theo[0],z_real[1]-z_theo[1]});
                dif(1) = turtlelib::normalize_angle(dif(1));
                arma::mat dis = (dif.t())*inv(dis_cov)*dif;
                distance(j) = dis(0,0);
            }
            std::cout << "i" << i<< std::endl;
            std::cout<< "distance"<<std::endl;
            distance.print();
            if(arma::min(distance)>new_landmark_thresh){
                //find a new landmark;
                landmark_lidar[this->cylinders_num] = i;
                this->cylinders_num+=1;
            }else{
                int min_idx = distance.index_min();
                int min_dis = distance.min();
                if(min_dis<landmark_mindis[min_idx]){
                    landmark_mindis[min_idx] = min_dis;
                    landmark_lidar[min_idx] = i;

                }              
            }
        }
    }
    ROS_INFO("after%d",this->cylinders_num);

    ROS_INFO("GET landmark");

    R = this->std_r[0] * arma::eye<arma::mat>(2*this->max_num,2*this->max_num);
    arma::mat H = arma::zeros<arma::mat>(2*this->max_num,2*this->max_num+3);
    arma::vec diff = arma::zeros<arma::vec>(2*this->max_num);

    for(int i=0;i<this->cylinders_num;i++){
        //k is the idx for lidar measurements
        int k = landmark_lidar[i];
        //meaning we only see 2 objects right now.
        if(k==-1)continue;
        // int i=k;
        std::cout <<"k"<<k<< "i" <<i << std::endl;
        float relative_x = msg.markers[k].pose.position.x;
        float relative_y = msg.markers[k].pose.position.y;
        // ROS_INFO("relativex %f",relative_x);
        // ROS_INFO("relative_y %f",relative_y);

        //relative distance, relative theta
        arma::vec2 z={sqrt(relative_x*relative_x+relative_y*relative_y),turtlelib::normalize_angle(atan2(relative_y,relative_x))};
        //new landmarks
        if(this->initialize_landmark[i] ==1){
            this->current_state(3+2*i,0) = this->current_state(1,0)+z[0]*cos(z[1]+this->current_state(0,0));
            this->current_state(3+2*i+1,0) = this->current_state(2,0)+z[0]*sin(z[1]+this->current_state(0,0));
            this->initialize_landmark[i] = 0;
        }
        (this->real_measurements)(2*i,0) = z[0];
        (this->real_measurements)(2*i+1,0) = z[1];

        //caluclate the theoretical_measurements;
        //mx - x_state;my-y_state
        relative_x =this->current_state(3+2*i,0)-this->current_state(1,0);
        relative_y =this->current_state(4+2*i,0)-this->current_state(2,0);
        double d= relative_x*relative_x+relative_y*relative_y;
        arma::vec2 z_theo={sqrt(d),turtlelib::normalize_angle(atan2(relative_y,relative_x))};
        (this->theoretical_measurements)(2*i,0) = z_theo[0];
        (this->theoretical_measurements)(2*i+1,0) = turtlelib::normalize_angle(z_theo[1]-this->current_state(0,0));

        //update the ekf
        //the derivate with respect to the state
        arma::mat Hj = arma::zeros<arma::mat>(2,3+2*this->max_num);
        Hj(arma::span(0,1), arma::span(0,2))=arma::mat({{0,-relative_x/sqrt(d),-relative_y/sqrt(d)},{-1,relative_y/d,-relative_x/d}});
        Hj(arma::span(0,1), arma::span(2*(i+1)+1,2*(i+1)+2))=arma::mat({{relative_x/sqrt(d),relative_y/sqrt(d)},{-relative_y/d,relative_x/d}});
        // arma::mat cov = this->covariance(arma::span(0,2+2*this->cylinders_num),arma::span(0,2+2*this->cylinders_num));
        arma::mat cov = this->covariance;
        this->H.rows(2*i,2*i+1) = Hj;
        //(2*n+3)*2
        // K = cov*H.t()*(H*cov*H.t()+R).i();
        // // arma::vec diff = z-z_theo;
        // diff.rows(2*i,2*i+1) = this->real_measurements.rows(2*i,2*i+1)-\
        // this->theoretical_measurements.rows(2*i,2*i+1);
        // diff(2*i+1) = turtlelib::normalize_angle(diff(2*i+1));
        // // this->current_state.rows(0,3+2*this->cylinders_num-1)+=K*diff;
        // this->current_state+=K*diff;
        // this->covariance= (arma::eye<arma::mat>(2*this->max_num+3,2*this->max_num+3)-K*H)*cov;
        // this->H.rows(2*i,2*i+1) = Hj;
    }
    ekf_update();
    // std::cout << "H" <<std::endl;
    // std::cout << Hj << std::endl;
    // std::cout <<"covariance" <<std::endl;
    // std::cout << this->covariance <<std::endl;
    // std::cout << "K" << std::endl;
    // std::cout << Hj <<std::endl;
    ROS_INFO("realm:%lf,%lf.%lf,%lf,%lf,%lf",real_measurements(0,0),real_measurements(1,0),real_measurements(2,0),real_measurements(3,0),\
                real_measurements(4,0),real_measurements(5,0));
    ROS_INFO("theom:%lf,%lf.%lf,%lf,%lf,%lf",theoretical_measurements(0,0),theoretical_measurements(1,0),theoretical_measurements(2,0),theoretical_measurements(3,0),\
                theoretical_measurements(4,0),theoretical_measurements(5,0));
    ROS_INFO("current_state:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",this->current_state(0,0),this->current_state(1,0),this->current_state(2,0),\
    this->current_state(3,0),this->current_state(4,0),this->current_state(5,0),\
    this->current_state(6,0),this->current_state(7,0),this->current_state(8,0));
    std::cout << std::endl;
    ROS_INFO("ekf_update");



    // this->ekf_update();
    //define the markerarray here
    visualization_msgs::MarkerArray markerArray;
    this->drawmarker(markerArray);
    this->vis_pub.publish(markerArray);

    //publish the green robot pose in the map frame
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = this->current_state(1,0);
    pose.pose.position.y = this->current_state(2,0);
    this->green_poses.push_back(pose);
    path.poses = this->green_poses;
    this->green_path_pub.publish(path);

    // this->transform_green();

}
/// \brief unknown data association
// void Message_handle::data_association(const visualization_msgs::MarkerArray& markers){
//     for (int i=0;i<)
// }
/// \brief extended kalman filter prediction
void Message_handle::ekf_predict(){
    //update the estimate using the model
    arma::mat A = arma::eye<arma::mat>(2*this->cylinders_num+3,2*this->cylinders_num+3);
    arma::mat update = arma::eye<arma::mat>(3,0);
    double last_theta = turtlelib::normalize_angle(this->current_state(0,0));
    double dtheta=accumulated_twist[0];
    double dx=accumulated_twist[1];

    //if no angular velocity in twist
    if (abs(dtheta)<0.001){
        this->current_state(1,0)+= dx*cos(last_theta);
        this->current_state(2,0)+= dx*sin(last_theta);
        A(1,0) -= dx*sin(last_theta);
        A(2,0) += dx*cos(last_theta);
    }
    else{
        this->current_state(0,0) += dtheta;
        this->current_state(1,0)+= (-dx/dtheta*sin(last_theta)+(dx/dtheta)*sin(last_theta+dtheta));
        this->current_state(2,0)+= (dx/dtheta*cos(last_theta)-(dx/dtheta)*cos(last_theta+dtheta));
        A(1,0) += (-dx/dtheta*cos(last_theta)+(dx/dtheta)*cos(last_theta+dtheta));
        A(2,0) += (-dx/dtheta*sin(last_theta)+(dx/dtheta)*sin(last_theta+dtheta));
    } 
    this->current_state(0,0)=turtlelib::normalize_angle(this->current_state(0,0));
    arma::mat Q = arma::zeros<arma::mat>(2*this->cylinders_num+3,2*this->cylinders_num+3);
    Q(0,0)= this->std_q[0];
    Q(1,1)=this->std_q[1];
    Q(2,2) = this->std_q[2];
    //update the coviance
    // arma::mat tmp = this->covariance(arma::span(0,2+this->cylinders_num),arma::span(0,2+this->cylinders_num));
    this->covariance(arma::span(0,2+2*this->cylinders_num),arma::span(0,2+2*this->cylinders_num))= \
    (A*this->covariance(arma::span(0,2+2*this->cylinders_num),arma::span(0,2+2*this->cylinders_num))*A.t())+Q;
    // this->covariance = A*(this->covariance)*(A.t())+Q;
    this->accumulated_twist = {0,0,0};

}
/// \brief extended kalman filter predication and update
void Message_handle::ekf_update(){
    int n = this->cylinders_num;
    //compute K
    // arma::mat R = this->std_r[0] * arma::eye<arma::mat>(2*this->cylinders_num,2*this->cylinders_num);
    // this->K = this->covariance*((this->H).t())*(this->H*this->covariance*((this->H).t())+R).i();
    // //posterior state update
    // arma::mat diff = arma::zeros<arma::mat>(2*this->cylinders_num,1);
    // diff = this->real_measurements-this->theoretical_measurements;
    // for(int i=0;i<this->cylinders_num;i++){
    //     diff(2*i+1)=turtlelib::normalize_angle(diff(2*i+1));
    // }
    // this->current_state+=this->K*diff;

    // this->current_state(0,0)=turtlelib::normalize_angle(this->current_state(0,0));

    // this->covariance = (arma::eye<arma::mat>(2*this->cylinders_num+3,2*this->cylinders_num+3)-this->K*this->H)*this->covariance;

    arma::mat R;
    if (this->std_r[0]<0.0000001)R = arma::eye<arma::mat>(2*n,2*n);
    else R = this->std_r[0] * arma::eye<arma::mat>(2*n,2*n);
    arma::mat cov = this->covariance(arma::span(0,2+2*n),arma::span(0,2+2*n));
    arma::mat n_H = this->H(arma::span(0,2*n-1),arma::span(0,2+2*n));
    this->K(arma::span(0,2*n+2),arma::span(0,2*n-1)) = cov*(n_H.t())*((n_H*cov*(n_H.t())+R).i());
    arma::mat diff = this->real_measurements-this->theoretical_measurements;
    for(int i=0;i<this->cylinders_num;i++){
        diff(2*i+1)=turtlelib::normalize_angle(diff(2*i+1));
    }
    this->current_state.rows(0,2*n+2)+=this->K(arma::span(0,2*n+2),arma::span(0,2*n-1))*diff.rows(0,2*n-1);
    this->current_state(0,0)=turtlelib::normalize_angle(this->current_state(0,0));
    this->covariance(arma::span(0,2*n+2),arma::span(0,2*n+2)) = (arma::eye<arma::mat>(2*n+3,2*n+3)-this->K(arma::span(0,2*n+2),arma::span(0,2*n-1))*n_H)*cov;
    
    //     if (this->std_r[0]<0.0000001)R = arma::eye<arma::mat>(2*this->max_num,2*this->max_num);
    // else R = this->std_r[0] * arma::eye<arma::mat>(2*this->max_num,2*this->max_num);
    // this->K = this->covariance*((this->H).t())*(this->H*this->covariance*((this->H).t())+R).i();
    // arma::mat diff = arma::zeros<arma::mat>(2*this->cylinders_num,1);
    // diff = this->real_measurements-this->theoretical_measurements;
    // for(int i=0;i<this->cylinders_num;i++){
    //     diff(2*i+1)=turtlelib::normalize_angle(diff(2*i+1));
    // }
    // this->current_state+=this->K*diff;
    // this->covariance = (arma::eye<arma::mat>(2*this->max_num+3,2*this->max_num+3)-this->K*this->H)*this->covariance;

    std::cout << "H" <<std::endl;
    std::cout << n_H << std::endl;
    std::cout <<"covariance" <<std::endl;
    std::cout << cov <<std::endl;
    std::cout << "K" << std::endl;
    std::cout << this->K(arma::span(0,2*n+2),arma::span(0,2*n-1)) <<std::endl;
    // ROS_INFO("realm:%lf,%lf.%lf,%lf,%lf,%lf",real_measurements(0,0),real_measurements(1,0),real_measurements(2,0),real_measurements(3,0),\
    //             real_measurements(4,0),real_measurements(5,0));
    // ROS_INFO("theom:%lf,%lf.%lf,%lf,%lf,%lf",theoretical_measurements(0,0),theoretical_measurements(1,0),theoretical_measurements(2,0),theoretical_measurements(3,0),\
    //             theoretical_measurements(4,0),theoretical_measurements(5,0));
    // ROS_INFO("current_state:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",this->current_state(0,0),this->current_state(1,0),this->current_state(2,0),\
    // this->current_state(3,0),this->current_state(4,0),this->current_state(5,0),\
    // this->current_state(6,0),this->current_state(7,0),this->current_state(8,0));
    std::cout << std::endl;

    //posterior covariance

}
/// \brief broadcast the transform from world frame to robot red/base_footprint frame
/// x,y,theta input the pose of the robot relative to the world frame
void Message_handle::transform(){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world" ;
    transformStamped.child_frame_id = "blue/base_footprint" ;

    //first set the translation
    transformStamped.transform.translation.x = this->diffdrive.body_pos().translation().x;
    transformStamped.transform.translation.y = this->diffdrive.body_pos().translation().y;
    transformStamped.transform.translation.z = 0.0;
    //set the rotation og the robot
    tf2::Quaternion q;
    q.setRPY(0, 0, this->diffdrive.body_pos().rotation());
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
    //publish the blue pose in the map frame
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = this->diffdrive.body_pos().translation().x;
    pose.pose.position.y = this->diffdrive.body_pos().translation().y;
    this->blue_poses.push_back(pose);
    path.poses = this->blue_poses;
    this->blue_path_pub.publish(path);

}
/// \brief broadcast the transform from map frame to robot green/base_footprint frame
/// x,y,theta input the pose of the robot relative to the world frame
void Message_handle::transform_green(){
    static tf2_ros::TransformBroadcaster br_green;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map" ;
    transformStamped.child_frame_id = "green/base_footprint" ;

    //first set the translation
    transformStamped.transform.translation.x = this->current_state(1,0);
    transformStamped.transform.translation.y = this->current_state(2,0);
    transformStamped.transform.translation.z = 0.0;
    // ROS_INFO("transform:%lf",transformStamped.transform.translation.x);

    //set the rotation og the robot
    tf2::Quaternion q;
    q.setRPY(0, 0, this->current_state(0,0));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br_green.sendTransform(transformStamped);
}
/// \brief subscribe to the joint states and publish the odometry message
/// \param msg: the joint state pos, velocity and name
void Message_handle::joint_state_callback(const sensor_msgs::JointState& msg){
    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom" ;
    odom.header.stamp = ros::Time::now();
    odom.child_frame_id = "green/base_footprint";
    if(msg.velocity.size()==0) turtlelib::Vector2D wheel_vel{0,0};
    else{turtlelib::Vector2D wheel_vel{msg.velocity[0],msg.velocity[1]};}
    turtlelib::Vector2D wheel_pos{msg.position[0],msg.position[1]};
    //avoid sudden change in pos at the very start
    if(this->first_joint_flag){
        turtlelib::Transform2D trans;
        diffdrive.set_body_pos(trans);
        diffdrive.set_wheel_pos(wheel_pos);
        this->first_joint_flag = 0;
    }
    this->diffdrive.FK_calculate(wheel_pos);
    turtlelib::Transform2D trans = {{this->diffdrive.body_pos().translation().x,this->diffdrive.body_pos().translation().y},\
                turtlelib::normalize_angle(this->diffdrive.body_pos().rotation())};
    this->diffdrive.set_body_pos(trans);
    geometry_msgs::Pose pos;
    pos.position.x = this->diffdrive.body_pos().translation().x;
    pos.position.y = this->diffdrive.body_pos().translation().y;
    pos.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, this->diffdrive.body_pos().rotation());
    pos.orientation.x = q.x();
    pos.orientation.y = q.y();
    pos.orientation.z = q.z();
    pos.orientation.w = q.w();
    odom.pose.pose = pos;

    geometry_msgs::Twist twist;
    odom.twist.twist.linear.x = this->diffdrive.bodytwist().xdot;
    odom.twist.twist.linear.y = this->diffdrive.bodytwist().ydot;
    odom.twist.twist.angular.z = this->diffdrive.bodytwist().thetadot;
    // odom_pub.publish(odom);
    
    //get the body twist directly from the odometry
    arma::vec3 twist2 = {this->diffdrive.bodytwist().thetadot,this->diffdrive.bodytwist().xdot,0};
    this->accumulated_twist+=twist2;

    static tf2_ros::TransformBroadcaster br_odom;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "green/base_footprint";
    //the rest transformation is set to zero automatically
    transformStamped.transform.translation.x =   this->diffdrive.body_pos().translation().x;
    transformStamped.transform.translation.y =  this->diffdrive.body_pos().translation().y;
    //set the rotation og the robot
    q.setRPY(0, 0, this->diffdrive.body_pos().rotation());
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br_odom.sendTransform(transformStamped);
}

/// \brief draw sets of cylinders with given start point and radius
/// markerArray input the holder of the definition of markers
void Message_handle::drawmarker(visualization_msgs::MarkerArray &markerArray){
    //make sure that every marker has unique id;
    int id = 0;
    for(unsigned int i=0;i<this->cylinders_num;i++){
    //the read marker pose
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    //set the different poses
    marker.pose.position.x = this->current_state(3+2*i,0);
    marker.pose.position.y = this->current_state(3+2*i+1,0);
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
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //add the marker to the array
    markerArray.markers.push_back(marker);
    id++;
    }
}
/// \brief publish the tf transform from map to odom
void Message_handle::map_odom_tf(){
    static tf2_ros::TransformBroadcaster br2;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";
    //the rest transformation is set to zero automatically
    turtlelib::Transform2D Tmr = turtlelib::Transform2D({this->current_state(1,0),this->current_state(2,0)},this->current_state(0,0));
    turtlelib::Transform2D Tor = this->diffdrive.body_pos();
    turtlelib::Transform2D Tmo = Tmr*(Tor.inv());
    transformStamped.transform.translation.x =  Tmo.translation().x;
    transformStamped.transform.translation.y =  Tmo.translation().y;
    //set the rotation og the robot
    tf2::Quaternion q;
    q.setRPY(0, 0,Tmo.rotation());
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br2.sendTransform(transformStamped);
}
int main(int argc, char ** argv){
    ros::init(argc, argv, "ekf");
    int cylinders_num;
    double radius,track;
    double cylinder_radius;
    double max_num;
    double new_landmark_thresh;
    std::vector<double> std_q;
    std::vector<double> std_r;

    ros::param::get("/max_num",max_num);
    ros::param::get("/std_q",std_q);
    ros::param::get("/std_r",std_r);
    ros::param::get("/wheel_radius",radius);
    ros::param::get("/track_width",track);
    ros::param::get("/cylinder_radius",cylinder_radius);
    ros::param::get("/new_landmark_thresh",new_landmark_thresh);

    Message_handle msgh;
    msgh.diffdrive.set_param(radius,track);
    msgh.first_joint_flag = 1;
    for(int i=0;i<max_num;i++){
        msgh.initialize_landmark.push_back(1);
    }
    msgh.max_num = max_num;
    msgh.cylinders_num = 0;
    msgh.new_landmark_thresh = new_landmark_thresh;

    msgh.current_state= arma::mat(2*max_num+3,1,arma::fill::zeros);
    // msgh.covariance = arma::zeros<arma::mat>(2*cylinders_num+3,2*cylinders_num+3);
    msgh.covariance = arma::mat(2*max_num+3,2*max_num+3,arma::fill::zeros);
    msgh.covariance(arma::span(0,2), arma::span(0,2)) = arma::zeros<arma::mat>(3,3);
    msgh.covariance(arma::span(3,2*max_num+2), arma::span(3,2*max_num+2))=\
        // std::numeric_limits<double>::infinity() * arma::eye<arma::mat>(2*cylinders_num,2*cylinders_num);
        10000 * arma::eye<arma::mat>(2*max_num,2*max_num);

    msgh.accumulated_twist = {0,0,0};
    msgh.H = arma::zeros<arma::mat>(2*max_num,2*max_num+3);
    msgh.K = arma::zeros<arma::mat>(2*max_num+3,2*max_num);
    msgh.real_measurements=arma::zeros<arma::mat>(2*max_num,1);
    msgh.theoretical_measurements =arma::zeros<arma::mat>(2*max_num,1);

    msgh.std_q = std_q;
    msgh.std_r = std_r;
    msgh.rate = 30;
    msgh.cylinder_radius = cylinder_radius;

    //fill all the markers 
    ros::NodeHandle n;
    // ros::Subscriber sensor_sub= n.subscribe("/fake_sensor",1000,&Message_handle::sensor_callback,&msgh);
    ros::Subscriber lidar_sub= n.subscribe("/clustering_circle",1000,&Message_handle::lidar_callback,&msgh);
    // ros::Subscriber lidar_sub= n.subscribe("/fake_sensor",1000,&Message_handle::lidar_callback,&msgh);

    msgh.odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);
    ros::Subscriber joint_sub = n.subscribe("/blue/joint_states",1000, &Message_handle::joint_state_callback,&msgh);    
    msgh.vis_pub = n.advertise<visualization_msgs::MarkerArray>( "/slam_obstacles", 0 ,true);
    msgh.blue_path_pub = n.advertise<nav_msgs::Path>("/blue/nav_msgs/Path", 1000);
    msgh.green_path_pub = n.advertise<nav_msgs::Path>("/green/nav_msgs/Path", 1000);

    ros::Rate r(msgh.rate);

    while (ros::ok())
    {
        //broadcast the transform from world frame to blus_basefootprint
        msgh.transform();
        //broadcast the transform from map to green_basefootprint
        msgh.map_odom_tf();
        // msgh.transform_green();
        ros::spinOnce();
        r.sleep();
    }

}