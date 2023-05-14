#!/bin/bash
export ROS_MASTER_URI=http://192.168.4.1:11311/
export ROS_IP=192.168.4.1
source ros_entrypoint.sh
source catkin_ws/devel/setup.bash
roslaunch local_planner local_planner_hw.launch
