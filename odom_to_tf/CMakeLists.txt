cmake_minimum_required(VERSION 3.0.2)
project(odom_to_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})