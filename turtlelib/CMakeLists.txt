cmake_minimum_required(VERSION 3.9)
project(turtlelib)


# The add_library function just added turtlelib as a "target"
# A "target" is a name that CMake uses to refer to some type of output
# In this case it is a library but it could also be an executable or some other items

find_package(catkin REQUIRED COMPONENTS roscpp)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS catch_ros roscpp
#  DEPENDS other non-ROS libs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
# create the executable target  and link it with the rigid2d library
# It is also possible specify multiple cpp files and they will be linked
# into a single executable (as long as exactly one of these files includes a main() function).
# However, by creating a library (as we are doing here) the library files
# can be compiled once and used
# create the turtlelib library 
add_library(turtlelib src/diff_drive.cpp src/rigid2d.cpp src/circle_detect.cpp)
# add_library(turtlelib src/diff_drive.cpp src/rigid2d.cpp)
target_link_libraries(${PROJECT_NAME} armadillo ${catkin_LIBRARIES})

add_executable(frame_main src/frame_main.cpp)
target_link_libraries(frame_main turtlelib)


install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Use the cmake testing functionality. A test is just an executable.
# enable_testing()
# add_executable(turtlelib_test tests/tests.cpp )
# target_link_libraries(turtlelib_test turtlelib)
# add_test(NAME Test_of_Turtlelib COMMAND turtlelib_test)
if(CATKIN_ENABLE_TESTING)
find_package(catkin REQUIRED COMPONENTS catch_ros)
catch_add_test(my_test tests/testros.cpp)
target_link_libraries(my_test	${catkin_LIBRARIES} ${PROJECT_NAME})
endif()
