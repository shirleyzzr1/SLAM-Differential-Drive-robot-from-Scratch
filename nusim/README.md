### README

#### package descripstion

This package provide a simulated robot environment, which visualize the cylinders at given start place and visulize a red turtle3-burger robot.

#### Launch file description

The launch file loads the rviz, joint_state_publier, joint_state_publisher_gui, and nusim node. Also, the yaml file for some basic configurations is loaded and xacro file for the robot is loaded.

#### parameters

In the yaml file: the current configuration is shown as below, you can change this before running the code.

```
robot_start: [-0.6,0.8,1.57]
radius: 0.038
cylinders_start:[[-0.6,-0.8],[0.6,-0.8],[0.6,0.8]]
```

other parameters which used in launch file can be set is shown as below.

```
  <arg name="use_rviz" default ="true" />  //whether to display rviz
  <arg name="use_jsp" default="true" />   //whether to use joint_state publisher
  <arg name = "~x0" default="0"/>    // the x initial position of the robot
  <arg name = "~y0" default="0"/>	 // the y initial position of the robot
  <arg name = "~theta0" default  = "0"/> the 
  <arg name = "~rate" default = "50"/> the initial rate of the robot
```

#### sample input

```
roslaunch nusim nusim.launch
```

