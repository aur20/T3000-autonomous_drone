THISD=${PWD}
cd src/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
# source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}: \
    ~/catkin_ws/src/avoidance/avoidance/sim/models: \
    ~/catkin_ws/src/avoidance/avoidance/sim/worlds
cd ${THISD}