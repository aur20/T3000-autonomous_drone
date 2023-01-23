FROM ros_initial:latest

# install ros packages for (AirSim), (rviz)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git ros-noetic-tf2-geometry-msgs ros-noetic-tf2-sensor-msgs \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c 'mkdir -p ~/catkin_ws/src/image_pipeline'
COPY ./p2c/image_pipeline /root/catkin_ws/src/image_pipeline
RUN /bin/bash -c 'cd ~/catkin_ws && catkin build image_pipeline'

COPY ./p2c/p2c2.launch /root/catkin_ws/src/image_pipeline/depth_image_proc_custom/launch/p2c2.launch
