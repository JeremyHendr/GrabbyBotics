# General information about the Mapping Workspace

---

**Project:** Grabby\
**Author:** Julius Ortstadt\
**Date:** 16.01.2026\
**Description:** General information about the workspace and usefull commands.

---

## Table of Contents
- [General information about the Mapping Workspace](#general-information-about-the-mapping-workspace)
  - [Table of Contents](#table-of-contents)
  - [Information](#information)
  - [Specifications](#specifications)
  - [Commands](#commands)

## Information
* Details on each package and setup can be found in the **Config.md** file of this directory. 
* Make sure that the **robot_control.ino** programm is loaded on the RaspberryPi.
* Make sure that the controller is connected to the RPi. In this case, it is via bluetooth.

## Specifications
**Development boards:** RaspberryPi 5 / Arduino Nano\
**ROS Distribution:** ROS1 Melodic in Docker container\

## Commands
- Workspace: `navigation_ws`
    - Make (at workspace root)
    ```
    catkin_make
    ```
    - Source (at workspace root)
    ```bash
    source devel/setup.bash
    ```
- Launch **Teleop package**
    - **Launch command:** (for default port configuration: /dev/ttyUSB0)
    ```bash
    roslaunch teleop_robot teleop.launch
    ```
    - **Launch command:** (for custom port configuration)
    ```bash
    roslaunch teleop_robot teleop.launch serial_port:=/dev/ttyACM0
    ```
**Note:** This launch command launches:
* the teleop package
* the rosserial connection
* the joy node


- Launch **Cartographer package**
    - **Launch command:** (for default port configuration: /dev/ttyUSB0)
    ```bash
    roslaunch cartographer_pkg mapping_basic.launch
    ```
    - **Launch command:** (for custom port configuration)
    ```bash
    roslaunch cartographer_pkg mapping_basic.launch serial_port:=/dev/ttyACM0
    ```
    - **Launch command:** Map visualisation
    ```bash
    roslaunch cartographer_pkg map_visualization.launch
    ```
**Note:** Replace *mapping_basic.launch* with *mapping_imu.launch* to map using the IMU as additional odometry information.

- **Saving the map:** (as a .pbstream). The result will be a *.pbstream* file.
```bash
rosservice call /write_state "filename: '/ros_shared/maps/map_name.pbstream'"
```
If an error appears at saving, verify that the */write_state* service is launched:
```bash
rosservice list
```
- **Converting the map:** To convert the *.pbstream* to a usefull ROS map.
```bash
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename /ros_shared/maps/map_name.pbstream -map_filestem /ros_shared/maps/map_name_ros
```

**Note:** The resolution of the resulting map can be specified as a parameter:
```bash
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename /ros_shared/mapping_ws/src/cartographer_pkg/maps/map_name.pbstream -map_filestem /name/of/path/map_name_ros --resolution 0.05
```
