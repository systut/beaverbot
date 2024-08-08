cmake_minimum_required(VERSION 2.8.3)
project(beaverbot_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy controller_manager joint_state_controller robot_state_publisher rviz std_msgs genmsg urdf xacro tf 
)

set(CMAKE_BUILD_TYPE Debug)

set(ROS_BUILD_TYPE Debug)
