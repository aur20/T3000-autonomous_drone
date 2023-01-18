# Offboard control example

**Set drone to HIL-mode**

Original sources of script: https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html

### What happens here?

PX4 HITL-simulation with ROS and RPI.

Preferred startup sequence:
 * Connect QGroundControl via RPI to flight controller
 * Start docker, run all containers except 
 * Connect drone via USB, start AirSim
 * Wait for GPS lock in QGroundControl
 * Run offboard_control
 * Anything might happen but the drone shall fly