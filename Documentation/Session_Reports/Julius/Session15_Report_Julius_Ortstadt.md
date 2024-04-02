# Session 15 Report - 02.04.2024 - Julius Ortstadt

## Before today's session
- I was working a lot on the LiDAR and the Jetson. 
This is why I created a documentation of all the steps I undertake as I progress throughout the project. 
I continued to write this documentation. 

- For the LiDAR to work I had to modify the ROS launch file provided with the LiDAR package.
Then you have to launch both roscore so that you have a master to manage all the topics, and the launch file itself in order to publish data to the other nodes.\
![Roscore](/Documentation/Session_Reports/Julius/Pictures/Session_15/roscore.png)\
![Roslaunch](/Documentation/Session_Reports/Julius/Pictures/Session_15/Launched_LiDAR_scan.png)

- I then had my first success with the LiDAR.
I could now visualize the data in the terminal (the raw data)\
![Raw LiDAR Data](/Documentation/Session_Reports/Julius/Pictures/Session_15/Raw_LiDAR_Data.png)\
and later in rviz.\
![Rviz visual](/Documentation/Session_Reports/Julius/Pictures/Session_15/rviz.png)
Rviz is a visualization software included in the ROS package which I used for the LiDAR.
Here is a live view:

https://github.com/JeremyHendr/GrabbyBotics/assets/120115242/fcc1d2d0-ea8c-4255-8718-83cb4c6e21bf

- Since the LiDAR was now correctly working with ROS, I had to setup the Hector SLAM. 
Hector (Heterogeneous Cooperating Team of Robots) SLAM is a SLAM algorithm that allows the robot to know where it is in a room without odometry data.
Meaning it knows where it is solely thanks to certain points in the LiDAR data.

- Now that Hector SLAM was installed, I needed a robot that can navigate a room without bumping into everything so that I could map the room.
I first tried to create a basic bluetooth remote with my phone and a HCSO6 bluetooth module. 
However the module wasn't working properly which led me to abandon this idea.
I instead coded a basic obstacle avoidance programm utilizing the from sonar and the Arduino.
I then performed some basic tests.

- Now that everything was ready I needed to integrate the Jetson and LiDAR with Grabby itself.
I had to improvise a little bit. 
Here is the test setup:\
![Test setup](/Documentation/Session_Reports/Julius/Pictures/Session_15/grabby_test_setup.jpg)  

- Everything was ready for the first mapping test. 
Here are two videos, one view is from rviz showing the realtime creation of the map and the other shows the robot navigating so that the map can be created. 
Note that the videos are sped up.

https://github.com/JeremyHendr/GrabbyBotics/assets/120115242/86aef2d8-5312-4246-a183-24707fe7e197

https://github.com/JeremyHendr/GrabbyBotics/assets/120115242/7278811c-f9ad-4bdc-be21-e6ae00db4dca

- You can see in the two videos, that the robot moves in the map. 
However the quality of the map is poor because the robot was moving to fast.
The speed of the robot is what needs fixing. 
However, this is the lowest I can set the motors so that they start. 
I will need to do some research into how to fix this. 
This is really important to get a high quality map.


- I also made some small design changes to the LiDAR case. 
I juste needed to move a few holes for the UART converter.\
![LiDAR case](/Documentation/Session_Reports/Julius/Pictures/Session_15/LiDAR_Case_Redesign.png)


## During today's session
- We had our progress presentation.
We presented everything that has been done since the last time. 
This included for the mobile base:
    - The frame
    - The LiDAR work
    - The Arduino Mega shield
    - The work with the Jetson
    - and much more

- I also started the real final version of the frame. 
It will take a total of two printing sessions for the entire frame to be finished.

- During my previous work I came to the conclusion that my Arduino / H-Bridge stack was too much of a hassle to work with. 
We came up with the idea of simply putting the H-Bridge vertical onto one of the side panels and just use spacer for the Arduino.

- Setting up the Jetson as headless, starting all the scripts and everything is a lot of work everytime the Jetson is turned on.
Therefore, I started to code a simple start-up script, which would execute everytime the Jetson starts.
This script configures the board for headless mode, sets the execution rights for the USB, launches roscore and the necessary launch files.

- I continued working with Hector SLAM trying to correct some errors in the map creation but I couldn't get it to work properly today.

- I continued my work on the documentation.


## For the next session
- I will finish printing the frame.
However for this frame to be assembled I will wait to have the base plate (made from plexi) ready, so that this would be the final assembly.

- If I have some time, I will start designing the plexi parts of the robot but this is not a priority yet.

- I will continue working on the virtual (Fusion 360) robot prototype so that all the holes I will need can me made with the laser cutter.

- I will continue writing my start-up script.

- I will also continue my work with Hector Slam.

- I will also look into how I can reduce the motor speed for better mapping performance.
