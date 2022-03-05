cmake_minimum_required(VERSION 3.0.2)
project(nuslam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(DEFAULT_BUILD_TYPE "Debug")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs message_generation nusim nuturtlebot_msgs roscpp std_msgs tf2 tf2_ros turtlelib
)


# Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES control.srv)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nuslam
  CATKIN_DEPENDS geometry_msgs message_runtime nusim nuturtlebot_msgs roscpp std_msgs tf2 tf2_ros turtlelib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(ekf src/ekf.cpp)
target_link_libraries(ekf armadillo ${catkin_LIBRARIES})

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ekf
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS
  src/ekf.cpp
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)