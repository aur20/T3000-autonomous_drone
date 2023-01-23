FROM ros_initial:latest

# install ros packages for (AirSim), (rviz)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git ros-noetic-tf2-geometry-msgs ros-noetic-tf2-sensor-msgs \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

# compile AirSim
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && \
    git clone https://github.com/Microsoft/AirSim.git && \
    cd AirSim && \
    ./setup.sh && \
    ./build.sh'

# compile AirSim ROS wrapper
COPY AirSim/airsim_ros_wrapper.cpp /AirSim/ros/src/airsim_ros_pkgs/src/airsim_ros_wrapper.cpp
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd AirSim/ros && \
    catkin build airsim_ros_pkgs'
# copy launch file
COPY AirSim/static_transforms.launch AirSim/ros/src/airsim_ros_pkgs/launch/static_transforms.launch
#COPY AirSim/p2c_mk1.launch /p2c_mk1.launch
#COPY AirSim/p2c_mk2.launch /p2c_mk2.launch
#RUN /bin/bash -c 'mv /p2c_mk1.launch AirSim/ros/src/airsim_ros_pkgs/launch/p2c_mk1.launch && \
#    mv /p2c_mk2.launch AirSim/ros/src/airsim_ros_pkgs/launch/p2c_mk2.launch'
