# Grabby - The Warehouse Robot
#### By Jeremy Hendrikse and Julius Ortstadt

## Introduction
This GitHub Repo holds all the necessary documents, codes and informations relative to the Grabby robot.\
This project has been in development for the last two years as part of our Master studies in Robotics and Autonomous Systems at Polytech Nice-Sophia.

> Note that the project is basically separated into two parts:
> - The mobile base which is responsible for moving the entire robot. 
> It also holds all the computational element of the robot, the battery etc.
> - The lifting platform which is responsible for reaching the package that is to be retrieved. 
> This platform can move up/down and forward/backward and uses different sensors to detect and retrieve the boxes.

## Code section
In this section you can find all the necessary codes that are needed to either achieve a specific function of the robot or to determine certain physical parameters of the robot. 
For instance, the mobile base has an Excel sheet that we created to determine the relationship between the robot's speed and the applied PWM to the motors.\
It also has all the necessary codes for mapping and navigation and other features.

## Hardware section
The Hardware section contains all the different 3D and 2D models that are necessary to build the robot from scratch.
Fusion360 and Inkscape were used to create these different objects.

## Documentation section
In this section, we grouped all of our research into how different this can and will be done on our robot as well as the components that we ordered for the robot itself.\
Also, you can follow the evolution of our robot through our session and other reports.

In the *Research* folder, you can find all the different research topics and some software necessary to operate certain functionalities of the robot.\
Additionally, a setup guide has been written. This document guides you through the setup process of the NVIDIA Jetson Nano so that you can operate the robot, this includes:
- ROS1 Melodic setup.
- Adding Arduino IDE and creating the connection between the two boards.
- Installing and activating NoMachine to use the Jetson in a headless mode.
- First use of the LiDAR
- Some ROS1 tips:
  - Creating a launch file
  - Creating a workspace
  - Working with ROS
- Some useful links and documentations.


## Thanks
We would like to thank our professors at Polytech Nice-Sophia for their valued support and insight along the development of this project. 