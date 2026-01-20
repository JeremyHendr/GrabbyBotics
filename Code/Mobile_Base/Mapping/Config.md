# Setup of the Mapping workspace for the Grabby project

---

**Project:** Grabby\
**Author:** Julius Ortstadt\
**Date:** 27.11.2025\
**Description:** Information about the setup and configuration of the mapping workspace for the Grabby robot.

---

## Table of Contents
- [Setup of the Mapping workspace for the Grabby project](#setup-of-the-mapping-workspace-for-the-grabby-project)
  - [Table of Contents](#table-of-contents)
  - [Information](#information)
  - [Specifications](#specifications)
    - [Information: Teleop package](#information-teleop-package)
    - [Information: Cartographer package](#information-cartographer-package)
  - [Setup](#setup)
    - [Workspace](#workspace)
    - [Teleop package](#teleop-package)
    - [Joystick package](#joystick-package)
    - [LiDAR package](#lidar-package)
      - [Testing in the terminal](#testing-in-the-terminal)
      - [Testing with RVIZ](#testing-with-rviz)
    - [Cartographer package](#cartographer-package)
      - [Installation](#installation)
      - [Package](#package)
    - [IMU package](#imu-package)
      - [Installation](#installation-1)
      - [Testing](#testing)
  - [References](#references)


## Information
The launch commands can be found in the **Info** file of this directory.
This file describes the configuration and setup of the various packages in the workspace to achieve mapping capabilities with the robot.

## Specifications
**Development boards:** RaspberryPi 5 / Arduino Nano\
**ROS Distribution:** ROS1 Melodic in Docker container\


### Information: Teleop package 
**Package Name:** teleop_robot\
**Node Name:** teleop_robot\
**Topics:**
  - Subscribes to */joy*
  - Published on */cmd_serial*

### Information: Cartographer package 
**Package Name:** cartographer_pkg
- **Launch files:**
  - *mapping_basic.launch* 
    - Load robot description (*grabby_robot.urdf*)
    - Start robot state publisher
    - Start RPLIDAR node (specify the serial port / default is set to *ttyUSB0*)
    -  Start Google Cartographer node with custom configuration file (uncomment the desired configuration in the *mapping.launch* launch file):
       -  Only LiDAR: *mapping_basic.lua*
       -  LiDAR + IMU: *mapping_imu.lua*
       -  LiDAR + IMU + Odometry: *mapping_imu_odom.lua*
  - *map_visualization.launch*
    - Start Rviz with custom configuration (*demo.rviz*)   

## Setup
### Workspace
- Create a folder
```bash
mkdir -p ./mapping_ws/src
cd mapping_ws
```
- Build the workspace
```bash
catkin_make
```
- Source the workspace
```bash
source devel/setup.bash
```
- Auto-source the workspace at container startup by adding the following to the **.bashrc**
```bash
nano ~/.bashrc
source /ros_shared/mapping_ws/devel/setup.bash
```


### Teleop package
In the previously created workspace, create a new package that will hold the necessary nodes for the teleoperation.
- Go to the source folder of the workspace
```bash
cd mapping_ws/src
```
- Create a new package with the necessary dependencies
```bash
catkin_create_pkg teleop_robot roscpp rospy sensor_msgs geometry_msgs
```
- Go to the source folder and add the **controller_teleop_node.cpp** file
```bash
cd teleop_robot/src
touch controller_teleop_node.cpp
```
- Go to the root of the package and modify the **CMakeLists.txt** by adding the following at the right place. The comments help localize where.
```cmake
add_executable(teleop_robot src/controller_teleop_node.cpp)

target_link_libraries(teleop_robot ${catkin_LIBRARIES})
```
- At the root of the package, create a new folder named **launched** which will hold the launch file for the teleoperation and create the **teleop.launch** file
```bash
mkdir launch
cd launch
touch teleop.launch
```

### Joystick package
Install the joystick package to control the robot using a PS4 dualshock 4 controller.

- Give information about the ROS repositories
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
- Install **curl** if not already done.
```bash
sudo apt install curl
```
- Add the keys
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
- Update
```bash
sudo apt update
```
- Install the package
```bash
sudo apt install ros-melodic-joy
```
- Run the node to test the installation.
```bash
rosrun joy joy_node
```

### LiDAR package
Install the RPLiDAR ROS package to use the LiDAR for mapping.
- Go to the **src** folder of the workspace
- Clone the repository
```bash
sudo git clone https://github.com/Slamtec/rplidar_ros.git
```
- Go back to the root of the workspace and build the workspace
```bash
catkin_make
```

#### Testing in the terminal 
- Start *roscore* in one terminal
- In another terminal, launch the LiDAR node. The default port is /dev/ttyUSB0. If another port has been assigned, use the argument: 
```bash
roslaunch rplidar_ros rplidar_a1.launch
```
- In a new terminal, the **/scan** topic should now be displayed when typing:
```bash
rostopic list
```
- The raw data can be displayed with:
```bash
rostopic echo /scan
```

#### Testing with RVIZ
To visualize the data in RVIZ, do the previous steps as for the terminal but instead of the *echo* command:
- Start rviz by typing *rviz* in the terminal
- Go to *Add* -> *By topic* -> *LaserScan*
- Under the topic, select */scan*
- Change the *fixed frame* from *map* to *laser*

### Cartographer package
#### Installation
To install Google cartographer, either use the prebuilt packages (done here) or build from source (described on the [Google cartographer documentation](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)).

- Run:
```bash
sudo apt-get install ros-melodic-cartographer ros-melodic-cartographer-ros ros-melodic-cartographer-ros-msgs ros-melodic-cartographer-rviz
```

#### Package
To create the package, run the following commands:
```bash
cd mapping_ws/src
catkin_create_pkg cartographer_pkg cartographer_ros roscpp sensor_msgs nav_msgs tf2_ros urdf
cd mapping_ws
catkin_make
source devel/setup.bash
```

### IMU package
#### Installation
- Connect the IMU following the [pinout](https://pinout.xyz/) of the Pi.

  | BNO055 Pin | Pi Pin Number            |
  |------------|--------------------------|
  | Vin        | 3.3V Power               |
  | GND        | Ground (GND)             |
  | SDA        | I2C SDA                  |
  | SCL        | I2C SCL                  |

- After booting the RPi5, test if the IMU is detected. The command should show the adress of the IMU.
```bash
sudo i2cdetect -r -y 1
```
- Install the libi2c-dev library
```bash
sudo apt-get install libi2c-dev
```
- In the **src** folder of the workspace, clone the package
```bash
git clone https://github.com/dheera/ros-imu-bno055.git
```
- Go back to the root of the workspace and build
```bash
catkin_make
```

#### Testing
- Install the ROS IMU plugin so that the IMU can be visualized in rviz
```bash
sudo apt-get install ros-melodic-rviz-imu-plugin
```
- In a new docker terminal
```bash
roslaunch imu_bno055 imu.launch
```
- Check if the **/imu/data** topic is there
```bash
rostopic list
```
- Start RVIZ
```bash
rviz
```
- Change the fixed frame to imu.
- Click the Add button in the bottom left.
- Click imu under rviz_imu_plugin. 
- Click OK.
- Change the topic of the imu to /imu/data.
- Move the BNO055 around, and the axes should move. 
  - The red line is the x-axis
  - The green line is the y-axis
  - The blue line is the z-axis.


## References
- ROS1 Melodic: Create a workspace
https://wiki.ros.org/catkin/Tutorials/create_a_workspace

- ROS1 Melodic: Create a package
https://wiki.ros.org/ROS/Tutorials/CreatingPackage

- RPLIDAR GitHub repository
https://github.com/Slamtec/rplidar_ros

- Google cartographer "build from source" documentation
https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html

- How to Publish IMU Data Using ROS and the BNO055 IMU Sensor
https://automaticaddison.com/how-to-publish-imu-data-using-ros-and-the-bno055-imu-sensor/

- ROS Standard Units of Measure and Coordinate Conventions
https://www.ros.org/reps/rep-0103.html

- RaspberryPi Pinout
https://pinout.xyz/

- Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055
https://www.adafruit.com/product/2472

- ROS1/ROS2 C++ driver for Bosch BNO055 IMU (I2C)
https://github.com/dheera/ros-imu-bno055