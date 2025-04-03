# Session 11 Report - 05.03.2024 - Julius Ortstadt

## Before today's session
Before today's session I made some modifications to some of the different code snippets.\
Then I also continued the assembly of the shield. 
However I ran into some problems. 
These mainly being faulty solder spots I needed to correct and a missing component. 
This led to me not being able to finish the assembly. 

## During today's session
During today's session:
-  I corrected some errors I made on the Mega Shield. 
I connected some of the solder spots that weren't correctly connected.

- After I made these corrections, I connected the rest of the data cables to the shield.\
![Connected Mega Shield](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_11/Shield.jpg)

- I also connected all the +5V and GND cables to the mini PCB I made for that purpose. 
As said previously, this allows for more space on the Shield and for better cable managment.\
![Connected mini PCB](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_11/Mini_PCB.jpg)

This mainly concludes the construction part of the mobil base of the robot. 
There are still some small things that need to be done in order to combine the two parts. 
However, for now, the code is the main point of focus until both parts are ready to be mounted together.\
![Finished construction part of the robot](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_11/Assembled.jpg)

On the code side of things: 
- After I wired everything up except the Ultrasonic sensors, I had to make some adjustments to some code snippets so that the right pins are read for each sensor.

- I then tested the IR sensors individually to see if they all work properly. 
Additionally, I had to change the detection distance on the sensors so that they see the line correctly.

- I encountered a problem which led to some difficulties with the line following. 
The bottom plate of the robot bent a little bit leading to the fact that the main propulsion wheels didn't have full contact with the ground. 
In order to correct this, I took off the spacer of one of the ball wheels. 
This compensated enough for the bend so that the wheels were touching the ground better.
Later on, we will maybe change from wood to plexi in order to avoid possible bends in the structure and allow for a better overall stability.

- I then re-tested basic motor movements just to see if everything worked well with the new cabling.

- After this was completed, I modified the code I wrote before this session to test some basic line following. 
These modifications included additional variables, the modification of pins, a better code structure and some return values.

- During the test I had a small problem with the center IR sensor which wouldn't detect the line correctly.
I didn't have enough time to correct this today.

## For the next session
For the next session:
- I will need to see why the center IR sensor didn't work properly.
- I will continue the main code.
However, the implementation of the LiDAR at this stage of the project could be benificial.
- I will start working on understanding how to use the LiDAR.

