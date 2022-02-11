cmake_minimum_required(VERSION 2.8.3)
project(nusim)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
        genmsg message_generation roscpp std_msgs tf2 tf2_ros turtlelib)

## Declare ROS messages and services
# add_message_files(DIRECTORY msg FILES Num.msg)
add_service_files(DIRECTORY srv FILES pose.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime roscpp std_msgs turtlelib
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/${PROJECT_NAME}.launch
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(PROGRAMS
  urdf/common_properties.xacro
  urdf/turtlebot3_burger.urdf.xacro
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/urdf
)
install(FILES
  config/basic_world.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/nusim.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)


