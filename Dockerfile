FROM ros:noetic-ros-base-focal
# From https://hub.docker.com/_/ros
# All subsequent files are generated from this file.
##############################################################

# This is an auto generated Dockerfile for ros:perception
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-base-focal

# install ros packages for (perception), (mavros and geographiclib), (rqt graph and topic), (catkin and catkin tools)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-perception=1.5.0-1* \
    ros-noetic-mavros ros-noetic-mavros-extras python3-geographiclib \
    ros-noetic-rqt-graph ros-noetic-rqt-topic ros-noetic-rqt-reconfigure \
    ros-noetic-catkin python3-catkin-tools \
    ros-noetic-tf2-geometry-msgs ros-noetic-tf2-sensor-msgs libpcl1 ros-noetic-octomap-* \
    wget git nano screen \
    && rm -rf /var/lib/apt/lists/* && rosdep update

# Install Avoidance dependencies
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh

# create catkin workspace
RUN mkdir -p /catkin_ws/src && cd /catkin_ws && catkin init

# download and compile AirSim
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd / && \
    git clone https://github.com/Microsoft/AirSim.git AirSim && \
    cd /AirSim && \
    rm -r ros2 && \
    ./setup.sh && \
    ./build.sh && \
    cd ros && \
    catkin build airsim_ros_pkgs --cmake-args -DCMAKE_BUILD_TYPE=Release'

# download Avoidance
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /catkin_ws/src && \
    git clone https://github.com/PX4/avoidance.git avoidance'

# download custom image_proc
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /catkin_ws/src && \
    git clone https://github.com/aur20/image_pipeline.git'

# compile the rest of the packages
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /catkin_ws && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release'

COPY airsim_node_custom.launch AirSim/ros/src/airsim_ros_pkgs/launch/airsim_node_custom.launch
COPY avoidance_hitl_airsim_mavros.launch /catkin_ws/src/avoidance/avoidance/launch/avoidance_hitl_airsim_mavros.launch
COPY avoidance_hitl_stereo.launch /catkin_ws/src/avoidance/avoidance/launch/avoidance_hitl_stereo.launch
COPY local_planner_hitl_airsim.launch /catkin_ws/src/avoidance/local_planner/launch/local_planner_hitl_airsim.launch

RUN apt-get update && apt-get install -y --no-install-recommends \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
    
