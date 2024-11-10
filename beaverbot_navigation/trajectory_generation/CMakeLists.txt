cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sdv_msgs
  tf
)

#find_package(Eigen3 REQUIRED)



set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/global_trajectory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/a_star.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utilities.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/coverage_planner.cpp
    )

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_generation/global_trajectory.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_generation/a_star.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_generation/utilities.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_generation/coverage_planner.h
    )

add_library(trajectory_lib ${SOURCES} ${HEADERS})
target_link_libraries(trajectory_lib ${catkin_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS EIGEN3
  CATKIN_DEPENDS roscpp
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(generate_trajectory src/generate_trajectory.cpp)
target_link_libraries(generate_trajectory ${catkin_LIBRARIES} trajectory_lib)

add_executable(a_star_node src/a_star_node.cpp)
target_link_libraries(a_star_node ${catkin_LIBRARIES} trajectory_lib)
