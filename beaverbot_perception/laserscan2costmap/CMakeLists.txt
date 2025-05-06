cmake_minimum_required(VERSION 2.8.3)
project(laserscan2costmap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp tf sensor_msgs nav_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(laserscan2costmap src/laserscan2costmap.cpp)
target_link_libraries(laserscan2costmap ${catkin_LIBRARIES})