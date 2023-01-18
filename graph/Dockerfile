FROM ros:noetic-ros-base-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-perception=1.5.0-1*
#    && rm -rf /var/lib/apt/lists/*

# install rqt graph
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-rqt-graph ros-noetic-rqt-topic ros-noetic-rqt-service-caller ros-noetic-rqt-publisher
#    && rm -rf /var/lib/apt/lists/*

# define env so I can run graphical apps
# https://marinerobotics.gtorg.gatech.edu/running-ros-with-gui-in-docker-using-windows-subsystem-for-linux-2-wsl2/
ENV LIBGL_ALWAYS_INDIRECT=0
ENV DISPLAY=host.docker.internal:0.0

# start container by sourcing ros_entrypoint.sh and running rqt_graph
# make sure VcXsrv is running
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    rqt"]
