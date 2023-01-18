FROM ros:noetic-ros-base-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-perception=1.5.0-1*
#    && rm -rf /var/lib/apt/lists/*

# start bash and ros
CMD ["/bin/bash", "-c", "roscore"]
