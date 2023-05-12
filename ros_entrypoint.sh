#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_ws/devel/setup.bash"
source "/AirSim/ros/devel/setup.bash"
service ssh start
exec "$@"