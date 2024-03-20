# Session 13 Report - 19.03.2024 - Julius Ortstadt

## Before today's session
- After connecting the RX on the LiDAR to an oscilloscope, I saw that it was indeed sending data through this port which seems counterintuitive since it is called RX.

- The first thing I tried to do was to read data from this module.
The idea was to read a data packet sent by the LiDAR with the help of the Arduino UNO. 
I knew that there would be a bottleneck by simply comparing the amount of data sent by the LiDAR and the speed of the Arduino.
As I said, I only wanted to read some data, decrypt it and have a comprehensive piece of information where I could find the distance and the angle of a point.
After a lot of research on Arduino Libraries I found one that specialized in reading data packets send to a serial port of the board.
For that, I needed to define a new serial port on the Arduino using the SoftwareSerial library. 
This essentially allows the creation a new serial communication port besides the standard D0 & D1 for RX & TX on the Uno which only has the one.
In the end, I indeed received some data from the LiDAR as a packet, however it was incomprehensible. 
The encoder from the PacketSerial library was not made for this type of data transmission which ultimately lead to it not displaying the data correctly.

- I also tried using the driver which I found but I wasn't very successful with this either.

After all these tests I concluded that I order to use this LiDAR I would need way more time to make it work properly and even then I wouldn't have a guarantee that it would work.
I therefore decided to use another LiDAR that was in storage but more on that in the next session of this report.
 
## During today's session
- After we made new plans about how we will merge the two parts of the robot, a few adjustments to the frame were needed. 
The main change was to ease installation. 
I replaced the previous holding mechanism for the wood panels. 
It was replaced by a system that would essentially just screw the panel onto the frame and therefore hold everything together.
This change reduced complexity in both assembly and printing since it would need less supports.
We then started to think about the connection between the two parts.
We will integrate a spacer system on top of the frame instead of just a normal spacer in order to minimize stress on the insert. 
The M5 insert will then be put on the top of the spacer where the other part will be mated to.
The last addition is not yet visible on the render:\
![New frame assembly 1](/Documentation/Session_Reports/Julius/Pictures/Session_13/New_Frame_1.png)\
![New frame assembly 2](/Documentation/Session_Reports/Julius/Pictures/Session_13/New_Frame_2.png)\
The reason why we need the spacers is that there will be the LiDAR between the two parts.
This will enable us to get a 2D map of the area from not to high above, about the height of a normal vacuum robot.

- I the started to use the new LiDAR A1 from Slamtec.
I connected all the necessary cables.
After reading the documentation, I installed the software provided by the manufacturer: Slamtec RoboStudio. 
The software allows the user to visualize all the data and even use SLAM algorithms and more.
However, before this could work on Windows 11, the driver for the signal converter board needed to be installed. 
Since the LiDAR sends data via UART, a UART to USB module was provided, but so that Windows could detect this module, a driver was needed to be installed. 
After everything was installed, everything worked properly and I could visualize the data.\
![Data from the LiDAR](/Documentation/Session_Reports/Julius/Pictures/Session_13/Slamtec_RoboStudio.png)

## For the next session
- I will continue my work with the LiDAR and try to use it with the NVIDIA Jetson since in order for the robot to work, the LiDAR will send data to the Jetson, which will process it and take a decision and send it to the Arduino which will eventually move the robot where it needs to be.

- I will continue the redesign of some of the pieces of the robot in order to accommodate the changes that will need to be done.
