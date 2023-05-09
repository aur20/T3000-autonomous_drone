# From https://hub.docker.com/_/ros
# All subsequent files are generated from this file.
##############################################################

# This is an auto generated Dockerfile for ros:perception
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-base-focal

# install ros packages for (perception), (mavros and geographiclib), (rqt graph and topic), (catkin and catkin tools)
# RUN /bin/bash -c 'echo "deb http://security.ubuntu.com/ubuntu xenial-security universe" | sudo tee -a /etc/apt/sources.list'
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-perception=1.5.0-1* \
    ros-noetic-mavros ros-noetic-mavros-extras wget python3-geographiclib \
    ros-noetic-rqt-graph ros-noetic-rqt-topic \
    ros-noetic-catkin python3-catkin-tools python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential\
    build-essential cmake git nano screen pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
    unzip v4l-utils \
    gcc-arm* protobuf-compiler \
    python-dev python-numpy \
    libgtk2.0-dev libcanberra-gtk* libopenblas-dev \
    libblas-dev liblapack-dev libv4l-0 \
    libpcl1 ros-noetic-octomap-* \
    && rm -rf /var/lib/apt/lists/* && rosdep update

# get Avoidance module
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh
RUN mkdir -p catkin_ws/src && cd catkin_ws && catkin init
RUN cd catkin_ws/src && git clone https://github.com/PX4/avoidance.git

# get Ultrasonic lidar sensor module
RUN cd catkin_ws/src && \
    git clone --single-branch --branch rpi_ros_ultrasonic https://github.com/aur20/T3000-autonomous_drone rpi_ros_ultrasonic
#    rosdep install smbus
# ROS noetic only knows about smbus but I tested only using smbus2
RUN python3 -m pip install smbus2

# build modules
RUN catkin build -w /catkin_ws
