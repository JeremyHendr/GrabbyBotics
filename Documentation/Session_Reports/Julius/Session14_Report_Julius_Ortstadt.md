# Session 14 Report - 29.03.2024 - Julius Ortstadt

## Before today's session
We had the chance to have our computer science class before this project session. 
During this class, we learned how to connect the Arduino and the NVIDIA Jetson Nano. 
This helped a lot with the overall progress of the project.

- First, I did my research into how the Jetson would work with the LiDAR, how it will analyze the data etc..
I found out that this would best work with ROS.
I then set out to start learning about ROS overall and how to use it, what it does etc...
I found some tutorials online and started working on the installation process and all the rest.

- However, before I installed ROS I had to add another feature to the Jetson. 
My goal was to use the Jetson as a headless device, meaning I would have the desktop as a remote desktop on my notebook, I could also use it with the keyboard and mouse from my notebook. 
In order to achieve this I had to install NoMachine on both my computer and the Jetson and then setup different things so that everything could work over Wi-Fi.\
![NoMachine on Windows detectin the NVIDIA Jetson Nano](/Documentation/Session_Reports/Julius/Pictures/Session_14/NoMachine.png)\
![Jetson Desktop](/Documentation/Session_Reports/Julius/Pictures/Session_14/Jetson_Desktop.png)\
I also installed JTOP on the Jetson so that I could monitor the performance, memory usage, CPU & GPU load etc.\
![JTOP](/Documentation/Session_Reports/Julius/Pictures/Session_14/jtop.png)

- I then installed ROS onto the Jetson so that I could work on the LiDAR. 
For this project, I decided to use ROS Melodic since it is very well documented and there are existing projects that use Melodic with the Slamtec A1M8 LiDAR that I have. 
The possibility of using examples was one of the key points to choosing this distribution since I don't really know how to use ROS very well and this would help me use and understand it. 
The installation process was pretty long because I installed the full version of this distribution so thaht I could have access to all the tools I would need.
![ROS Installed](/Documentation/Session_Reports/Julius/Pictures/Session_14/ROS_Installed.png)

- In parallel to all this I also started writing a documentation of all the steps I undertook for NoMachine and ROS to work properly so that later on others as well as I can come back to the project and understand how everything was done.

- I continued my research on mapping a room with a LiDAR using Hector-SLAM. 
Achieving this is the next goal for this project.

- I made some changes to the frame that I didn't get to do last time. 

## During today's session
The base for today's session was laid thanks to the work done before this class.

- I tested the angular drift of the LiDAR, in other words, I tested what the angle of the laser was relative to a horizontal plane. 
This was necessary so that I could see how deep I could lower the LiDAR into a plane.
I had the value given by the manufacturer but I wanted to check so that there would be no unexpected surprise.
![Manufacturer data](/Documentation/Session_Reports/Julius/Pictures/Session_14/Scan_Flatness.png)

- I designed a holding system for the LiDAR to put in between the two parts of the robot without taking up to much space.
The piece needed to fit the LiDAR itself, the UART to USB converter and the cables.
The whole piece will rest on the overlap that will be screwed onto the top plate of the frame.\
![LiDAR box](/Documentation/Session_Reports/Julius/Pictures/Session_14/LiDAR_Case.png)

- I continued working on the frame. 
I corrected some small mistaked that I overlooked last time.
Additionaly I started working on the pilars that will hold together both parts of the project.\
![New started frame](/Documentation/Session_Reports/Julius/Pictures/Session_14/Frame.png)\
As I already said, the frame is not finished yet.

## For the next session
- During the next session we will hold a presentation about the progress since the last time we made an update. 
This will allow us to show what has been done, what we changed, why and what still needs to be done.

- I will try to interface the LiDAR with the Jetson and to get the data from the module and display them.

- I will try to make the robot map a room using the LiDAR and a basic object avoidance system with the Arduino and a proximity sensor (ultrasonic sensor) so that the robot could move around the room.
