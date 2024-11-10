cmake_minimum_required(VERSION 3.0.2)
project(beaverbot_pose)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geonav_transform
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geonav_transform 
                sensor_msgs 
                geometry_msgs 
                nav_msgs 
                roscpp 
                rospy 
                tf
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
###########
# Install #
###########

catkin_install_python(PROGRAMS 
  scripts/beaverbot_pose_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
