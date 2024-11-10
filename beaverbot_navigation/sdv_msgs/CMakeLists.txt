cmake_minimum_required(VERSION 2.8.3)
project(sdv_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  tf
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  Trajectory.msg
  TrajectoryPoint.msg
)

add_service_files(
  FILES
  TrajectoryFlags.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sdv_msgs
)

catkin_package(
DEPENDS EIGEN3
CATKIN_DEPENDS message_runtime
)

