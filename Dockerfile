# From https://hub.docker.com/_/ros
# All subsequent files are generated from this file.
##############################################################

# This is an auto generated Dockerfile for ros:perception
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-base-focal

# install ros packages for (perception), (mavros and geographiclib), (rqt graph and topic), (catkin and catkin tools)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-perception=1.5.0-1* \
    ros-noetic-mavros ros-noetic-mavros-extras wget python3-geographiclib \
    ros-noetic-rqt-graph ros-noetic-rqt-topic \
    ros-noetic-catkin python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh

# adjust global settings
# see https://discuss.ardupilot.org/t/rtt-too-high-for-timesync-with-sitl-mavros/38224/8
RUN /bin/bash -c "sed -i '12s/10.0/0.0/g' /opt/ros/noetic/share/mavros/launch/px4_config.yaml"

# compile example package
# this is done in one step for I need bash and don't want to source the script every time
COPY offb_node.cpp /offb_node.cpp
COPY own_cmake.txt /own_cmake.txt
COPY app.launch /app.launch
RUN mkdir -p ~/catkin_ws/src
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ~/catkin_ws/src && \
    catkin init && \
    catkin_create_pkg app roscpp mavros_msgs && \
    mv /offb_node.cpp ~/catkin_ws/src/app/src/offb_node.cpp && \
    mkdir ~/catkin_ws/src/app/launch && \
    mv /app.launch ~/catkin_ws/src/app/launch/app.launch && \
    cat /own_cmake.txt >> ~/catkin_ws/src/app/CMakeLists.txt && \
    cd ~/catkin_ws && \
    catkin build'

