cmake_minimum_required(VERSION 3.0.2)
project(beaverbot_localization)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/calibrate
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)