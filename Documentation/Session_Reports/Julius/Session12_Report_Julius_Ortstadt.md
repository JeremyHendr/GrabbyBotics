# Session 12 Report - 12.03.2024 - Julius Ortstadt

## Before today's session
- As indicated in the previous session report, I started gathering some knowledge about LiDAR technology in general since I've never worked on it before.
- I also tried to find out why the center IR sensor didn't work properly.
However, this wasn't my main point of focus. 
I wanted to get the LiDAR operational as this would greatly simplify the project and reduce the amount of sensors needed.

## During today's session
- Since I already started to learn about LiDAR, I continued my research on this subject but more specifically on the particular LiDAR that I had.
The LDS-02 LiDAR is a spare part for a robot vacuum but can also be used as a standalone 2D LiDAR.

- I tried finding information about this module but this was easier said than done. 
I found some docs about it from a company called ROBOTIS. 
Besides docs for the LiDAR they also had a GitHub repo in which was a driver for the module.
However understanding how to connect the module to the Arduino or even the PC was the main problem.

- In order to connect the LiDAR, I had to remove the pre-mounted connector since I didn't have the male-cable for it. 
I then soldered some cables to it with a connector at the end for which I had a fitting cable.\
![New connector](/Documentation/Session_Reports/Julius/Pictures/Session_12/LiDAR_Connector.jpg)

- The next step was about finding out how to receive data from the module. 
The problem was that the LiDAR doc had 4-pins: +5V, GND, TX, RX. 
However, the module that we had, had: +5V, GND, RX, PWM.
So we needed to find out how to communicate with the module.

- After continuing my research about this LiDAR I found out that the ROBOTIS team, in order to use the module with an Arduino, used a board manager in the Arduino IDE called OpenCR.
In order to install this board, I first had to find an ...index.json URL for this board since as a third-party board it isn't pre-loaded in the IDE.
After a lot of search I finally found what I was looking for and after some more research into how to actually load it into the IDE, I did just that.
Now I could use an example code provided with the OpenCR board.
The problem was just that the code was for displaying the data on a LCD screen shield for the Arduino.
I will need to figure out the code to modify it so that I can maybe display it in the IDE or another way.

- There still was the problem with figuring out the driver. 
There was no information about how to use it with the LiDAR.
We tried to understand the code but we still have some difficulties especially since the ports on the LiDAR don't correspond with the ones in the doc.
We will try to see how the LiDAR communicates via an oscilloscope.
If this doesn't work out we will maybe have to order another LiDAR built to be used with Arduino and/or Nvidia Jetson.
But we are not giving up yet.


## For the next session
- I will connect the RX port of the LiDAR to an oscilloscope in order to see what type of signal is sent by the module.

- I will continue to try to read data from the LiDAR and try to understand how it communicates.

- I will try to figure out how to use the driver provided by ROBOTIS.
