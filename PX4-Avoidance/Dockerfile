FROM ros_initial:latest

# install ros packages for (Avoidance), (rqt_reconfigure) --- fails for some reason
RUN apt update && apt-get install -y --no-install-recommends \
    git libpcl1 ros-noetic-octomap-* \
    ros-noetic-rqt-reconfigure \
    && rm -rf /var/lib/apt/lists/*

# compile AirSim
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ~/catkin_ws/src && \
    git clone https://github.com/PX4/avoidance.git avoidance'

# copy local_planner_custom.launch and compile PX4-Avoidance
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin build -w ~/catkin_ws/avoidance --cmake-args -DCMAKE_BUILD_TYPE=Release'

COPY PX4-Avoidance/local_planner_mk1.launch /local_planner_mk1.launch 
RUN /bin/bash -c 'mv /local_planner_mk1.launch ~/catkin_ws/src/avoidance/local_planner/launch/local_planner_mk1.launch'
