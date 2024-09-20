cmake_minimum_required(VERSION 3.0.2)
project(tsnd5_driver)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

# Catkin specific configuration
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
  scripts/tsnd5_driver_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)