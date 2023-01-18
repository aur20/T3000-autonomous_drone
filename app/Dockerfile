FROM ros:noetic-ros-base-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-perception=1.5.0-1*
#    && rm -rf /var/lib/apt/lists/*

# install mavros
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-mavros \
    ros-noetic-mavros-extras
#    && rm -rf /var/lib/apt/lists/*

# install build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-catkin \
    python3-catkin-tools

# create local app
COPY offb_node.cpp /offb_node.cpp
COPY own_cmake.txt /own_cmake.txt
COPY app.launch /app.launch
COPY offb_node.py /offb_node.py
RUN mkdir -p ~/catkin_ws/src
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ~/catkin_ws/src && \
    catkin init && \
    catkin_create_pkg app roscpp mavros_msgs && \
    catkin_create_pkg offboard_py rospy && \
    mv /offb_node.cpp ~/catkin_ws/src/app/src/offb_node.cpp && \
    mkdir ~/catkin_ws/src/app/launch && \
    cp /app.launch ~/catkin_ws/src/app/launch/app.launch && \
    cat /own_cmake.txt >> ~/catkin_ws/src/app/CMakeLists.txt && \
    cd ~/catkin_ws && \
    catkin build'
#    source devel/setup.bash && \
#    roscd offboard_py && \
#    mkdir launch && mkdir scripts && cd scripts && \
#    mv /offb_node.py ~/catkin_ws/src/offboard_py/scripts/offb_node.py && \
#    chmod +x offb_node.py && \
#    cp /app.launch ~/catkin_ws/src/offboard_py/launch/app.launch'

# run local app
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
   source ~/catkin_ws/devel/setup.bash && \
   roslaunch --wait app app.launch"]
