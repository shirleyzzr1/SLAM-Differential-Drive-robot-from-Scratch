# SLAM Differential Drive Robot
This project creates the ros packages and libraries for kinematics of differential drive robot control, EKF-SLAM, circle detection and data association.
# Package List
This repository consists of several ROS packages
- **nuturtle_descripton** - contains urdf files and basic debugging,testing, and visualization code for the robots using
- **nuturtle_control** - contains the interface to convert the different wheel_velocity and body twist command, calculate  the odometry of the robot
- **nusim** This package provides a simulated robot environment, which visualize the 
  cylinders at given start place and visulize a red turtle3-burger robot.
- **nuslam** This package uses extended kalman filter method to fuse the data from odometry and other measurements to 
estimate the state of the robot.



