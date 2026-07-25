cmake_minimum_required(VERSION 3.0.2)
project(beaverbot_cartographer)

find_package(catkin REQUIRED)

catkin_package()

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)