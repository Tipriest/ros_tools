cmake_minimum_required(VERSION 3.0.2)
project(joystick_to_cmdvel)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(joystick_to_cmdvel_node src/joystick_to_cmdvel_node.cpp)

## Specify libraries to link the executable target against
target_link_libraries(joystick_to_cmdvel_node
  ${catkin_LIBRARIES}
)
