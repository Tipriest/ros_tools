cmake_minimum_required(VERSION 3.0.2)
project(gt_tf_pub)

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

add_executable(gt_tf_pub src/gt_tf_pub.cpp)
target_link_libraries(gt_tf_pub ${catkin_LIBRARIES})