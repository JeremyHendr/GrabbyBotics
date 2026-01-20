# General information about the Navigation Workspace

---

**Project:** Grabby\
**Author:** Julius Ortstadt\
**Date:** 17.01.2026\
**Description:** General information about the workspace and usefull commands.

---

## Table of Contents
- [General information about the Navigation Workspace](#general-information-about-the-navigation-workspace)
  - [Table of Contents](#table-of-contents)
  - [Information](#information)
  - [Specifications](#specifications)
  - [Commands](#commands)


## Information
* Details on each package and setup can be found in the **Config.md** file of this directory. 
* Make sure that the **motor_controller_diff_drive_2.ino** programm is loaded on the RaspberryPi. It can be found in the *Base_controller* folder.


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
- Launch **Navigation stack**
    - **Basic launch command:** with 
      - Default `map_file`: /ros_shared/maps/map.yaml
      - Default `serial_port_arduino`: /dev/ttyUSB0
      - Default `serial_port_lidar`: /dev/ttyUSB1
    ```bash
    roslaunch navstack_pub nav.launch
    ```
    - **Launch command:** with arguments (all or some can be specified)
    ```bash
    roslaunch navstack_pub nav.launch map_file:=/ros_shared/maps/map_d103_ros.yaml serial_port_arduino:=/dev/ttyUSB0 serial_port_lidar:=/dev/ttyUSB1
    ```
**Note:** The map needs to be a ROS map (a *.yaml* and a *.pgm* file / specify the *.yaml* file in the parameter). The instructions for mapping and conversion can be found in the documentation for the mapping procedure.