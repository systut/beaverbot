#!/bin/sh
set -e
basic_dep="git \
           curl \
           nano \
           vim \
           python3-catkin-tools \
           python3-pip 
	  "

ros_dep="ros-noetic-tf \
         ros-noetic-robot-state-publisher \
         ros-noetic-rviz \
         ros-noetic-roslint \
         ros-noetic-nmea-navsat-driver \
         ros-noetic-xacro \
         ros-noetic-ros-control \
         ros-noetic-ros-controllers \
         ros-noetic-rviz-imu-plugin \
         ros-noetic-imu-filter-madgwick \
         ros-noetic-robot-localization \
         ros-noetic-map-server \
         ros-noetic-navigation"

python_dep="rospy-message-converter \
         python-socketio
         python-socketio[client]"

apt-get update
apt-get upgrade -y
apt-get install -y $basic_dep
DEBIAN_FRONTEND=noninteractive apt-get install -y $ros_dep
apt-get autoremove -y
apt-get clean -y
python3 -m pip install $python_dep
python3 -m pip install setuptools --upgrade
python3 -m pip install importlib_metadata --force-reinstall
