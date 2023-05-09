# Ultrasonic sensor to ROS PCL-pointcloud

Publish sensor data from ultrasonic sensor as PCL-pointcloud in ROS environment. This project is complementary to the Arduino project in [this repository](https://github.com/aur20/T3000-autonomous_drone/tree/arduino_sensors).

Part list:

 * Arduino Nano
 * 4 * [Ultrasonic Ranging Module HC - SR04](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
 * RPI reading data via I2C via Python
 * Jumper cables -> wire up as described in the top lines of Arduino script
 * **No additional resistor required**

## Usage

Activate I2C interface on RPI via `sudo raspi-config` -> Interfacing Options -> I2C -> Enable.

Then, either you have a local ROS Noetic installation on your RPI:

```
python3 -m pip install smbus2 # required for I2C communication
cd ~/catkin_ws/src
git clone --single-branch --branch rpi_ros_ultrasonic https://github.com/aur20/T3000-autonomous_drone rpi_ros_ultrasonic # this repository
```

Or you already have cloned and checkout to this branch, then build the dockerfile:

```
docker build . -t ros_ultra
docker run -it --rm --net=host --privileged ros_ultra
```

## Run the scripts:

To run the script, you'll need to `source ~/catkin_ws/devel/setup.bash` and have a `roscore` up and running. Then, run the script:

```
source ~/catkin_ws/devel/setup.bash
rosrun rpi_ros_ultrasonic sensor_interface.py
```

