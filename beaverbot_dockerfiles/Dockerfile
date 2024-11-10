FROM osrf/ros:noetic-desktop-full

# copy install scripts
COPY ./beaverbot_dockerfiles/install /root/install

# update and install dependencies
RUN chmod +x /root/install/*

RUN /bin/sh -e -c /root/install/i_dep.sh

# copy directory specific to scart
COPY ./ /root/catkin_ws/src/

# set default bash shell
SHELL ["/bin/bash", "-c"]

# set environment variable for ros
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc

# install ros dependencies
WORKDIR /root/catkin_ws 

RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build ROS packages
RUN catkin config --skiplist beaverbot_driver

RUN . /opt/ros/noetic/setup.bash && \
    catkin build && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
