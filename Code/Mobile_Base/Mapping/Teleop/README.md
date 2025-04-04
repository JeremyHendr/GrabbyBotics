# Information about the package
#### Author: Julius Ortstadt

## Overview
Custom package for teleoperation of the robot during mapping.
Runs in the terminal.
SSH connection to Jetson and if the code on the Arduino needs to be updated, start a NoMachine session and push it through the Jetson desktop.\
Timeout function: if no input for 3 seconds, the robot stops. 

## Specifications
- **Development boards:** NVIDIA Jetson Nano / Arduino Mega
- **ROS Distribution:** ROS1 Melodic
- **Package Name:** teleop_custom 
- **Node Name:** teleop_node
- **Build package and ws:** at root of ws run:
```
catkin_make
```
- **Source ws:** at root of ws run:
``` 
source devel/setup.bash
``` 
- **Used topics:** 
  - Publishing: */teleop_timeout*
  - Arduino subscribes to: */teleop_timeout*
- **Launch command:** (for default port configuration: /dev/ttyUSB0)
```
roslaunch teleop_custom teleop.launch
```
- **Launch command:** (for custom port configuration)
```
roslaunch teleop_custom teleop.launch serial_port:=/dev/ttyACM0
```
- **Launch file:**
  - Starts teleop_node
  - Starts rosserial with Arduino on ttyUSB0 (default) at 115200 baudrate

- **Source files:**
Two source files are provided. 
One has the timeout function and the other doesn't.