# Session 16 Report - 09.04.2024 - Julius Ortstadt

## Before today's session
- I finished printing the new frame.
The only thing left to do now is to place the inserts and assemble it.

- After some research into how we can reduce the motor speed for mapping etc. , we came to the conclusion that it would be better to order other motors with a higher reduction ratio.
This would allow the motor to drive at lower speeds.

- I continued my work on the Jetson by installing the Arduino IDE on it and performing some basic tests to check the connection (via USB) between the two.\
![IDE on the Jetson](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_16/ide_jetson.png)

- I tested the connection by using a little script allowing me to make the built-in LED on the Arduino blink.

- I wrote a script that counts the pulses send by the motor encoder. 
This will later be usefull to determine the speed of the robot and its position.\
![Ticks](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_16/one_revo_tick_count.png)

- After testing this script, I had some problems with one of the encoders which didn't seem to work.
I will have to get back to that issue later.

- I then started catkin workspace for the robot in which we'll put all our code relative to ROS.

- I tested the publising manually:\
![Manually Published data](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_16/published_data.png)

- I started writing the launch file for this project which initially allowed the Arduino to publish, on a ROS topic, the encoder ticks from the motors.\
![Publishing via the launch file](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_16/ticks_published.png)


- I then added the LiDAR compatibility to the project which I setup during the last sessions.

- I added the LiDAR to the launch file which worked fine.
However I will need to figure out which USB port is given to which module so that they have the right references in the launch file when multiple devices are connected via USB.

- I wrote a script for the Arduino which will allow me to control it through the jetson via a GUI provided by ROS.
The script included publishing Twist messages (messages containing linear and angular velocity data), calculating the resulting PWM for the motors, applying the calculated PWM, determining the wheel velocity, correcting the velocity of each wheel so that the trajectory is what is asked...

- After all this I also continued my work on the documentation which retraces all the steps I made to setup the Jetson for this project.

## During today's session
- We made some fast calculations of our power consumption on the entire project. 
This enabled us to get a rough size for the battery.
We will probably need a battery with a capacity of a little more than 5000mAh, a voltage of 14.4V and a good enough discharge rate.

- Since our battery will give out 14.4V, we will need a DC-DC converter. 
We determined the specs of the converter we needed and started testing it.
We chose a Buck-Boost converter.

- Also, since we will have the Jetson running SLAM and a QR code recognition via vision, it will probably heat up a lot.
For that matter, we decided to add fans to the robot.
One on the top part to cool the H-Bridge.
And two in the mobile base: 
    - One for the Jetson directly on the heatsink.
    - One below the top panel, blowing air through the entire base.

- I designed an adapter for the H-Bridge so that it could fit onto the side panel.\
![Adapter](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_16/Adapter.png)

- I started incorporating those different design changes in the 3D model of the robot.
This simultaneously allowed me to place the holes etc. for the panels so that the creation of the DXF files for the laser cutter will be easier.\
![New robot design](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_16/New_Design.png)

## For the next session
- I will continue and maybe finish the 3D design of the robot.

- I will start printing the new pieces I need.

- I will continue my code for the navigation.
