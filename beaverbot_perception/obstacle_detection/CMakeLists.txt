cmake_minimum_required(VERSION 3.0.2)
project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES obstacle_detection

CATKIN_DEPENDS rospy std_msgs #edge_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/obstacle_detection_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Install ##
#############
