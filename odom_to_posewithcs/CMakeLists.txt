cmake_minimum_required(VERSION 3.0.2)
project(odom_to_posewithcs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(odom_to_posewithcs_node src/odom_to_posewithcs_node.cpp)

## Specify libraries to link the executable target against
target_link_libraries(odom_to_posewithcs_node
  ${catkin_LIBRARIES}
)
