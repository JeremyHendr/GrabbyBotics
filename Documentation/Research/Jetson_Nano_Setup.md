# Jetson Nano - LiDAR Work Documentation
#### Author: Julius Ortstadt

## Note
The information about how to write the code for the autonomous robot is presented in part in this document but also in the relevant part of the code section.\
For the mapping process, a different algorithm has been used as described here.\
This file should be use to setup the jetson itself and the dependencies and not for coding the actual robots actions.

## Introduction
This document will show step-by-step how I prepared the NVIDIA Jetson Nano for different tasks for the Grabby Warehouse robot project.

It is supposed that the Jetson Nano already has an OS installed and was setup. This document only shows different parts which group different tutorials found online.

## Remote desktop using NoMachine
In order to remotely connect the Nano with my PC so that I could see the desktop without any wired connection to another monitor I followed the tutorial made by [JetsonHacks](https://jetsonhacks.com/2023/12/03/nomachine-jetson-remote-desktop/).

Here are some [NoMachine installation tips](https://kb.nomachine.com/AR02R01074).

*Note*:
During the installation, you will be asked to [disable the xserver](https://forums.developer.nvidia.com/t/easy-way-to-change-a-default-nano-setup-to-work-in-headless-mode-ie-no-gui-services/107268). 
There are two ways to do it:
- Permantly
```
sudo systemctl set-default multi-user.target
```
- Temporarily
```
sudo systemctl isolate multi-user.target
```

If you chose the temporarily option, you will need to run the command everytime the Jetson is rebooted.
You then need to run this command (also everytime if you choose the temporarily option) which will restart the nxserver:
```
sudo /usr/NX/bin/nxserver --restart
```

However if you want to go back and un-disable the xserver, run the following command:
```
sudo systemctl set-default graphical.target
```

I also decided to [install JTOP](https://jetsonhacks.com/2023/02/07/jtop-the-ultimate-tool-for-monitoring-nvidia-jetson-devices/) in order to monitor the Jetsons performance.

*Important information*\
I haven't been to change the keyboard layout on the Jetson Nano. 
Therefore, I need to write the text in need on my computer and then copy/paste.

## ROS Melodic installation
For the ROS installation I followed a great tutorial made by [AutomaticAddison](https://automaticaddison.com/how-to-install-ros-melodic-on-the-nvidia-jetson-nano/) which I found on the [NVIDIA Developper Forum](https://forums.developer.nvidia.com/t/autonomous-mobile-robot-navigation-with-ros-jetson-nano-arduino-lidar-extended-kalman-filter/189802).

Here is also the link to the [official ROS wiki installation tutorial](http://wiki.ros.org/melodic/Installation/Ubuntu).


Additional information on how to source a [ROS Project](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment):
```
$ source /opt/ros/melodic/setup.bash
```
    
## First LiDAR Test
Now that we installed ROS on the Jetson. 
Let's setup our LiDAR so that we can see everything on the desktop.

I will base this work on the following tutorial:\
[*How to Build an Indoor Map Using ROS and LIDAR-based SLAM*](https://automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/#Connect_Your_RPLIDAR)

### Connect the LiDAR
1. Connect the LiDAR to the USB
2. Run: 
```
ls -l /dev | grep ttyUSB
```
3. Change the permission:
```
sudo chmod 666 /dev/ttyUSB0
```

### Create a catkin workspace and visualize the LiDAR data in the terminal
1. Update the Jetson
```
sudo apt-get update
```
2. Install the dependencies
```
sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev python-rosinstall python-rosinstall-generator python-wstool build-essential git
```

3. Create the [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
I chose to do it in a folder name "LiDAR_Test".
```
mkdir -p ~/LiDAR_Test/catkin_ws/src
```
```
cd ~/LiDAR_Test/catkin_ws/
```

4. Let's modify the .bash file so that we don't have to source the setup.bash file every time we open a new linux terminal.
Open a new terminal and type:
```
nano ~/.bashrc
```

5.  Verify that these two lines are at the end of the file:
```
source /opt/ros/melodic/setup.bash
source ~/LiDAR_Test/catkin_ws/devel/setup.bash
```

6. Type the following command to source the file:
```
source ~/LiDAR_Test/catkin_ws/devel/setup.bash
```
7. Go back to the src folder and clone the RPLIDAR ROS package to this folder:
```
sudo git clone https://github.com/Slamtec/rplidar_ros.git
```
8. Go to the root of the workspace and compile it:
```
catkin_make
```

9. Source the file:
```
source ~/LiDAR_Test/catkin_ws/devel/setup.bash
```

10. In another terminal window launch the Roscore
```
roscore
```
11. In the window where we have our catkin_ws, launch the package (I am working with the A1 lidar, so the command that I need to use is the one below. If you use another LiDAR you need to check the README.md file in the git repo of the RPLIDAR)
```
roslaunch rplidar_ros rplidar_a1.launch
```
12. If everything was successful, let's see our active topics:
```
rostopic list
``` 

13. We should see a */scan* topic. 
If this is the case, everything went fine. 
Type the following to see the raw LiDAR data in the terminal:
```
rostopic echo /scan
```

### Visualize the data with rviz
In order to do this we need to connect to the Jetson via NoMachine which we setup earlier. 
Do all the step until you are on the desktop of the LiDAR.
You need to have launched the package like we did before for these steps.

1. Open a new terminal window and launch rviz
```
rviz
```

2. In *Global Options* change the **Fixed Frame** to **laser**

3. Click the **Add** button.

4. Select **LaserScan** and click **OK**

5. Set the **LaserScan** topic to **/scan**

6. Increase the size to **0,03** (for better visibility)

If you see everything correctly that means that everything is working great.

### Build a map using the Hector-SLAM ROS package
Now we want to build a map using our LiDAR. To do this we need to setup hector_slam for our robot.\
[Here is the link to the official ROS wiki for hecto_slam](http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot)

One important thing before we start this process. 
Since I work through Wi-Fi, the quality of the NoMachine stream is not the best.
I therefore connect with NoMachine only for the rviz part.
For the rest I just open multiple windows terminals in which I connect via ssh and execute the commands.
Here is how I structure it:
- One ssh terminal with roscore
- One ssh terminal where I will launch the launch file
- One ssh terminal where I usually setup headless mode, set the rights for the usb ...
- One connection via NoMachine to connect to the desktop where I launch rviz.
    
#### Install Qt4
Qt4 is a software that is used to generate graphical user interfaces.
```
sudo apt-get install qt4-qmake qt4-dev-tools
```
#### Download Hector-SLAM
1. Go to the catkin workspace's source folder
```
cd ~/LiDAR_Test/catkin_ws/src
```

2. Now we clone the git:
```
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
```
#### Set the coordinate frame parameters
We now need to set the frame names and options correctly. 
For more information about Coordinate Frames see the blog post by [AutomaticAddison](https://automaticaddison.com/coordinate-frames-and-transforms-for-ros-based-mobile-robots/) and the [official ROS wiki](http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot).

To do this follow the steps below:
1. Go to the launch file for Hector-SLAM:
```
sudo gedit ~/catkin_ws/src/hector_slam/hector_mapping/launch/mapping_default.launch
```

2. Look for the following lines:
```
<arg name="base_frame" default="base_footprint"/>
<arg name="odom_frame" default="nav"/>
```
And change them to this:
```
<arg name="base_frame" default="base_link"/>
<arg name="odom_frame" default="base_link"/>
```

3. Do the same thing with the following. Change this:
```
<!--<node pkg="tf" type="static_transform_publisher" name="map_nav_broadcaster" args="0 0 0 0 0 0 map nav 100"/>-->
```
to this
```
<node pkg="tf" type="static_transform_publisher" name="base_to_laser_broadcaster" args="0 0 0 0 0 0 base_link laser 100"/>
```

4. Save the file and return.
5. Go here:
```
cd ~/LiDAR_Test/catkin_ws/src/hector_slam/hector_slam_launch/launch
```

6. Open the tutorial.launch file. I will be using nano for the editor.
```
nano tutorial.launch
```

7. Find the following line:
```
<param name="/use_sim_time" value="true"/>
```
and change it to this:
```
<param name="/use_sim_time" value="false"/>
```

8. Save and return.
9. Build the packages with **catkin_make** when you are in **~/LiDAR_Test/catkin_ws/**


In my case I had an error making it impossible for the **catkin_make** to work.
The error was *Project ‘cv_bridge’ specifies ‘/usr/include/opencv’ as an include dir, which is not found. It does neither exist as an absolute directory nor in…*.
I found a fix that worked for me:
```
cd /usr/include
```
```
sudo ln -s opencv4/ opencv
```
After doing this, try building the packages again.
Normally it should work now.

If you want you can do a reboot of the Jetson.

#### Launch the mapping process
Since we did a reboot of the Jetson we need to [set it up for headless mode again](#remote-desktop-using-nomachine) (if you chose the temporary method).

Now let's launch the RPLIDAR.
It's important to note that you need to have roscore launched in another terminal window.

Type the following commands:
```
cd ~/LiDAR_Test/catkin_ws/
```
```
sudo chmod 666 /dev/ttyUSB0
```
```
roslaunch rplidar_ros rplidar_a1.launch
```

On the desktop of the Jetson, open a new terminal and type the following command. 
This will start the mapping process as well as starting rviz.
```
roslaunch hector_slam_launch tutorial.launch
```

In order for the mapping to work correctly, you need to move the LiDAR slowly across the room.

#### Save the map
There are multiple ways to save a map.
Here is the first one:
1. In the terminal (in my case one via ssh) type the following:
```
rostopic pub syscommand std_msgs/String "savegeotiff"
```

2. When the mapping is done, hit CTRL+C

3. You can find your map in *~/LiDAR_Test/catkin_ws/src/hector_slam/hector_geotiff/maps*



## Connect Arduino and Jetson
The next step for this project is to connect the Arduino with the Jetson.

### Download, install and setup the IDE
Since we are on the Jetson we can simply use a terminal.

```
git clone https://github.com/JetsonHacksNano/installArduinoIDE.git
```
```
cd installArduinoIDE
```
```
./installArduinoIDE.sh
```

This is an older version of the IDE but it still works very well.

Reboot the jetson with **sudo reboot**.
You will now see a desktop shortcut for the IDE. 
(I had to click on it and say that I trust it.)

Now shut everything down and plug the Arduino into one of the USB ports.
Boot everything up again.
When in the IDE select the board and the poart that you are using.
If you're not sure about the port, open a new terminal and type the following:
```
ls -l /dev | grep ttyUSB
```
This will show you all the devices connected via USB and therefore the port.
Select the right one in the IDE and you're ready to go.
I used an **Arduino Mega 2560** on port **/dev/ttyUSB0**

*Test:*
Go to **File -> Examples -> 01.Basics -> Blink**.
When you upload this code, the LED on the Arduino should blink if everything was done right.

### Integrate Arduino with ROS
The official guide can be found [here](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup).

Here is what I did:
1. Open a new terminal and install the following things:
```
sudo apt-get install ros-melodic-rosserial-arduino
```
```
sudo apt-get install ros-melodic-rosserial
```

2. Note the location of the sketbook. (**File -> Preferences**).
3. Close the IDE and go to the sketchbook location through a terminal.
4. Go to the *libraries* directory and type the following:
```
rm -rf ros_lib
```
5. Still in this directory:
**(With the period at the end)**
```
rosrun rosserial_arduino make_libraries.py .
```
6. With **dir** you should now see the *ros_lib* folder.

You can now experiment with some examples from this library.
Some more explanation can be found in the *Integrate Arduino With ROS * section of [this](https://automaticaddison.com/how-to-publish-wheel-encoder-tick-data-using-ros-and-arduino/#Integrate_Arduino_With_ROS) tutorial.

## Create the launch file for the project
In this part I will start creating the launch file for the entire project.

This will take bits of different sections of this doc. 

Let's start:
1. Create a location for the project:
```
mkdir -p ~/Grabby/catkin_ws/src
```
2. Go to this location and make the project:
```
cd Grabby/catkin_ws/
```
```
catkin_make
```
3. Create a folder in the *src* folder:
```
cd src
```
```
mkdir grabby
```
4. Move to the newly created folder and create the package:
```
catkin_create_pkg launch_grabby rospy roscpp std_msgs sensor_msgs geometry_msgs tf
```
5. Go to launch_grabby
6. Create a new folder. 
I will name it *launch*
7. Go to the main folder of the catkin workspace:
```
cd ~/Grabby/catkin_ws
```
8. Compile the package
```
catkin_make --only-pkg-with-deps launch_grabby
```
9. Source the catkin workspace.
```
source ~/Grabby/catkin_ws/devel/setup.bash
```
10. Mode inside the package
```
roscd launch_grabby
```
11. Edit the CMakeLists.txt file.
```
nano CMakeLists.txt
```
12. Remove the "#" on line 5 to enable C++11 support.
Then save and close.
13. Go to the launch folder and create a new launch file.
This is the file that will hold all the information about nodes etc. for the entire project.
```
cd launch
```
```
nano grabby.launch
```

### Add the Arduino Node to the launch file
Here we are going to add the node for Arduino to our launch file.
1. Add the following to the file (some things may vary depending on how everything is setup):
```
<launch>
  <node pkg="rosserial_python" type="serial_node.py" name="serial_node">
    <param name="port" value="/dev/ttyUSB0"/>
    <param name="baud" value="115200"/>
  </node>
</launch>
```
2. Save, close and change the permissions.
```
sudo chmod +x grabby.launch
```
3. Open a new terminal and launch the file.\
*Note: Roscore must be active in another terminal.*
```
roslaunch launch_grabby grabby.launch
```
4. When you type **rostopic list** in another terminal, you should be able to see your active topics, as well as access the data being published.

### Add the LiDAR Node to the launch file
In this part we will publish the LiDAR data. 
I used [this tutorial](https://automaticaddison.com/how-to-publish-lidar-data-using-a-ros-launch-file/).
1. Get the port of the LiDAR and change its permissions.
2. Go to the *src* folder of our workspace and clone the following git:
```
sudo git clone https://github.com/Slamtec/rplidar_ros.git
```
3. Go to the root of the workspace and do **catkin_make**

**Note**
If you haven't added the launch file to the .bashrc file, do it now so that you don't have to source it everytime:
```
cd ~
```
```
nano /.bashrc
```
Add the following lines:
```
source /opt/ros/melodic/setup.bash
source ~/Grabby/catkin_ws/devel/setup.bash
```
4. Go to the where our launch file is located. 
In this case:
```
cd ~/Grabby/catkin_ws/src/grabby/launch_grabby/launch
```
5. We need to install the ros package for the LiDAR.
Go to the root of the workspace.
```
catkin_make --only-pkg-with-deps rplidar_ros
```
6. In the launch file, add the following:
```
<!-- Lidar Data Publisher using the RPLIDAR A1 from Slamtec -->
<!-- Used for mapping and obstacle avoidance -->
<!-- Publish: /scan -->
<node pkg="rplidar_ros" type="rplidarNode" name="rplidarNode" output="screen">
  <param name="serial_port" value="/dev/ttyUSB0" type="string"/>
  <!-- For RPLIDAR A1 Baudrate is 115200 -->
  <param name="serial_baudrate" value="115200" type="int"/>
  <param name="frame_id" value="laser" type="string"/>
  <param name="inverted" value="false" type="bool"/>
  <param name="angle_compensate" value="true" type="bool"/>
</node>  
```

7. In another terminal, run **roscore**
8. In the other terminal, we will now launch our file:
```
roslaunch launch_grabby grabby.launch
```
9. In this case, there will be interference with the part of the file for the Arduino.
Just comment it.
We'll get back to this later.
10. Launch the file.
Type **rostopic list**. 
You should now see the */scan* topic. 
If you type *rostopic echo /scan* you will see the data being published.
11. If you want to visualize the data via rviz, you can do the steps in this [section](#visualize-the-data-with-rviz).
Just run **rosrun rviz rviz** in the terminal.


### How to Control a Robot’s Velocity Remotely Using ROS
I followed [this tutorial](https://automaticaddison.com/how-to-control-a-robots-velocity-remotely-using-ros/).

The code can be found in this github.
I is called "jetson_motor_control.ino".
You may need to adjust some of the values depending on the specs of the robot.
We will use **rqt** to drive the robot.

1. Start a **roscore** in a terminal.
2. Run the following command in another one to start the serail node between the Jetson and the Arduino:
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```
The value of the port may need to be changed according to the wiring of the robot.

3. In another terminal run this:
```
rosrun rqt_robot_steering rqt_robot_steering
```
4. You should now see data being published onto the **/cmd_vel** topic when you change the values on the slider.


**Note:**\
You can add this to the main launch file.
Just add the following to the launch file:
```
<node pkg="rqt_robot_steering" type="rqt_robot_steering" name="rqt_robot_steering">
</node>
```

### How to Create an Initial Pose and Goal Publisher in ROS
I followed [this tutorial](https://automaticaddison.com/how-to-create-an-initial-pose-and-goal-publisher-in-ros/).

Here is a quick guide:
1. First we need to create a package:
```
cd ~/Grabby/catkin_ws/src/grabby/
```
```
catkin_create_pkg localization_data_pub rospy roscpp std_msgs tf tf2_ros geometry_msgs sensor_msgs nav_msgs
```
2. Let's compile the package:
```
cd ~/Grabby/catkin_ws/

catkin_make --only-pkg-with-deps localization_data_pub 
```
3. Go inside the package and enable C++11 support (remove "#" on line 5).
```
cd ~/Grabby/catkin_ws/src/grabby/localization_data_pub/

nano CMakeLists.txt
```
4. For more information as to why we need to create a code which converts inputs from rviz to a different format take a look at the tutorial linked above.
5. Put the code named "rviz_click_to_2d.cpp" from this github into a file with the same name at this location:
**cd ~/Grabby/catkin_ws/src/grabby/localization_data_pub/src**
6. Type: **cd ..** to go back one folder. 
When there, add the following to the CMakeLists.txt file:
```
INCLUDE_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(/usr/local/lib)
 
add_executable(rviz_click_to_2d src/rviz_click_to_2d.cpp)
target_link_libraries(rviz_click_to_2d ${catkin_LIBRARIES})
```
7. Compile the code:
```
cd ~/Grabby/catkin_ws

catkin_make --only-pkg-with-deps localization_data_pub
```
8. Let's try the code (in different terminals):
```
roscore
```
```
rosrun localization_data_pub rviz_click_to_2d
```
```
rviz
```
```
rostopic echo /initial_2d
```
```
rostopic echo /goal_2d
```
9. In rviz, when you click the 2D Pose Estimate button, click on the map and do the same with the 2D Nav Goal, you should see something in the two topics that are echoed in the terminal.


**Note:**\
We can add this to the main launch file.
```
<!-- Initial Pose and Goal Publisher -->
<!-- Publish: /initialpose, /move_base_simple/goal -->
<node pkg="rviz" type="rviz" name="rviz">
</node>
<!-- Subscribe: /initialpose, /move_base_simple/goal -->
<!-- Publish: /initial_2d, /goal_2d -->
<node pkg="localization_data_pub" type="rviz_click_to_2d" name="rviz_click_to_2d">
</node>   
```

### How to Publish Wheel Odometry Information Over ROS
I followed [this tutorial](https://automaticaddison.com/how-to-publish-wheel-odometry-information-over-ros/).

Here is a quick guide (for more details, see the tutorial linked above):
1. Go to the source folder of our localization package and create a new C++ file:
```
cd ~/Grabby/catkin_ws/src/grabby/localization_data_pub/src

nano ekf_odom_pub.cpp
```
2. Into this file, put the code named *ekf_odom_pub.cpp* from this github repo.
3. Add the programm to the CMakeLists.txt file:
```
cd ~/Grabby/catkin_ws/src/grabby/localization_data_pub/

nano CMakeLists.txt
```
4. Add the following to this file:
```	
add_executable(ekf_odom_pub src/ekf_odom_pub.cpp)
target_link_libraries(ekf_odom_pub ${catkin_LIBRARIES})
```
5. Compile the workspace:
```
cd ~/catkin_ws

catkin_make --only-pkg-with-deps localization_data_pub
```
6. Let's test it out:
  - Start ros with **roscore**
  - Launch the node in another terminal
  ```
  rosrun localization_data_pub ekf_odom_pub
  ```
  - Start the tick counter publisher by connecting to the arduino. The code "jetson_motor_control.ino" should still be uploaded on the arduino. **Note: Change the port if necessary.**
  ```
  rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
  ```
  - In another terminal launch the initial pose and goal publisher:
  ```
  rosrun localization_data_pub rviz_click_to_2d
  ```
  - Start rviz
  - If you set the initial pose and the nav goal with rviz you should see something being published in */odom_data_quat*.

**Note:**\
You can add this to the main launch file:
```
<!-- Wheel Odometry Publisher -->
<!-- Subscribe: /right_ticks, /left_ticks, /initial_2d -->
<!-- Publish: /odom_data_euler, /odom_data_quat -->
<node pkg="localization_data_pub" type="ekf_odom_pub" name="ekf_odom_pub">
</node> 
```






## Other links
- Setup and Configuration of the Navigation Stack on a Robot
http://wiki.ros.org/navigation/Tutorials/RobotSetup

- How to Create and Execute ROS Launch Files
https://automaticaddison.com/how-to-create-and-execute-ros-launch-files/

The next steps are in this order:
- How to Create and Execute ROS Launch Files
https://automaticaddison.com/how-to-create-and-execute-ros-launch-files/

- Next is Coordinate Frames and Transforms for ROS-based Mobile Robots
https://automaticaddison.com/coordinate-frames-and-transforms-for-ros-based-mobile-robots/
Make the robot use TF: http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF

- Publishing Sensor Streams Over ROS (LaserScans or PointClouds):
http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors


- 3D rotation matrix with python code:\
https://automaticaddison.com/how-to-describe-the-rotation-of-a-robot-in-3d/

