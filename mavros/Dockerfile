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


# install GeographicLib
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget python3-geographiclib
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh 

# set ip of RPI
# ENV rpi_ip 192.168.1.193
RUN /bin/bash -c "sed -i '12s/10.0/0.0/g' /opt/ros/noetic/share/mavros/launch/px4_config.yaml"
# run mavros ... be sure to start simulator AirSim, RPI, and roscore node first
#CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
#    roslaunch --wait mavros px4.launch fcu_url:=tcp://${rpi_ip}:5760"]
