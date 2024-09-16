cmake_minimum_required(VERSION 3.8)
project(beaverbot_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs std_msgs tf message_generation
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/beaverbot_driver/robot_data.cpp
    )

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/beaverbot_driver/robot_data.h
)

add_library(industrial ${SOURCES} ${HEADERS})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RobotWheelVel.msg
  EncoderWheelVel.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs std_msgs tf message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(robot_communication src/beaverbot_driver/robot_communication.cpp)
target_link_libraries(robot_communication ${catkin_LIBRARIES} serial industrial)  
add_dependencies(robot_communication beaverbot_driver_generate_messages_cpp)

add_executable(encoder_to_odom src/beaverbot_driver/encoder_to_odom.cpp)
target_link_libraries(encoder_to_odom ${catkin_LIBRARIES} serial industrial) 
add_dependencies(encoder_to_odom beaverbot_driver_generate_messages_cpp)

add_executable(command_to_odom src/beaverbot_driver/command_to_odom.cpp)
target_link_libraries(command_to_odom ${catkin_LIBRARIES} serial industrial) 
add_dependencies(command_to_odom beaverbot_driver_generate_messages_cpp)
