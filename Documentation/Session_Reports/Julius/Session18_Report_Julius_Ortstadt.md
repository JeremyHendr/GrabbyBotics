# Session 18 Report - 07.05.2024 - Julius Ortstadt

## Before today's session
- I continued the 3D model of the robot so that we have a complete virtual view of it.
- For this model to be complete I needed to create a few other models of some components. 
In this case of the switch which we'll use.\
![Switch 3D model](/Documentation/Session_Reports/Julius/Pictures/Session_18/Switch.png)
- Now that I had everything for the virtual assembly, I integrated the components and finished the back panel design.\
![Back panel](/Documentation/Session_Reports/Julius/Pictures/Session_18/Back_Panel.png)
- Online I found a 3D model of the exact shield for the Arduino Mega that I was using, this allowed me to add it to the 3D model. 
This enabled me to see how everything would fit inside the robot.
- I had a problem during the last session where the encoder for one motor. 
After a few tests the problem was solved. 
The issue was a faulty cable.
- I made some progress on the assembly of new robot, which will be the final version.
- After this part of the assembly, another problem occured. 
The motors didn't work. 
The problem was another faulty wire responsible for the power supply. 
After replacement the motors worked properly.
- I then did a motor check through the Arduino connected to the Jetson nano via USB.
- I continued my code which would allow me to send velocity commands to the Arduino with the help of ROS.
I needed to troubleshoot a lot of things for this to work since some of the connection didn't work properly and there were also some other problems related to the encoders, the interrupts, the usb connection etc.
I managed to solve all of these problems and I started integrating this programm into the launch file I created for this project.
- I put the inserts for fixation of different parts into the robot. 
These were for the top panel, the back panel, the left side panel and the merge points for when we connect both parts of the project.
- I started a new print for the components that weren't printed correctly last time.
- I also designed and printed some custom spacers for the jetson nano.\
![Custom spacers](/Documentation/Session_Reports/Julius/Pictures/Session_18/Custom_spacer.png)
- I made some changes to the top panel to help with cable managment.
- I cut the back panel and the top panel so that I could start assembly.
- I installed the fan on the top panel.

## During today's session
- I swapped the old motors for the new ones.
These will allow the robot to move slower while increasing torque in order to create a good map of the warehouse and to increase precision.
- I made some small corrections to the back panel as some of the holes were too small.\
![Back panel fixed](/Documentation/Session_Reports/Julius/Pictures/Session_18/Back_Panel_Fixed.jpg)
- I fixed the Jetson nano to the robot. 
I couldn't do this earlier because I didn't have the right spacers (I needed to print them).\
![Installed Jetson nano](/Documentation/Session_Reports/Julius/Pictures/Session_18/Jetson.jpg)
- I gathered some components for the final steps of assembly.
- I re-routed some of the cables for better cable managment.
- I added the LiDAR case to the top panel.\3
![Installed top panel](/Documentation/Session_Reports/Julius/Pictures/Session_18/Top_Panel.jpg)
- I worked on the power connections for the project.
I did a quick sketch and will later start soldering everything together.

## For the next session
- I will finish the assembly of the robot.
- I will continue to work on the software for the navigation.
- I will add some final touches to the mobile base.
