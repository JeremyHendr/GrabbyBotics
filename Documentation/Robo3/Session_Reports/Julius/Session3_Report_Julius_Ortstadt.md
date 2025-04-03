# Session 3 Report - 08.12.2023 - Julius Ortstadt


Before today's session I continued the creation of 3D models for different parts of the robot.
The main part being the frame that will later be fixed onto the robot in order to add stability and ease assembly.\
![Frame](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_3/Frame.png)

Additionally, I also made some small changes to other models in order to take into consideration other factors of which I hadn't thought of before.

During today's session my focus was on getting the motors wired up and operational and to do some basic tests. 
However, before I could do that I needed to gather some components (Arduino Mega, power supply, cables, spacers, screws) and prepare mounting points to which I would later mount the H-Bridge as well as the Arduino itself.\
![Assembly](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_3/Assembly.jpg)

I also did some fit checks with a part that I printed. Its goal is to mount the IR sensor on the bottom side of the robot. However, due to some issues, the part needs to be modified a little bit. Here is the new version that I redesigned: \
![Modified IR sensor spacer](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_3/IR_sensor_spacer.png)

After that I connected the power to the robot and made some small movement tests which were a little bit complicated due to the short wire of the power supply. I wrote the code for the test with help from the manufacturer of the H-Bridge as well as the different schematics needed in order to correctly wire all the components. The main problem that I encountered, and that uterly took the most time to fix was with the motors. A wheel hub keeps unscrewing itself from the motor shaft due to vibrations. In order to refix it, the entire motor assembly has to be removed. Next session I will try to find a more permanent solution to the problem. Another part that took me a considerable amount of time was to wire up and use the motors since the schematics were a little bit difficult to understand at first.


I also started a new order list in which I listed some components I will need later on:
- A battery 
- A fuse with fuse holder, as it was encouraged to use one by the manufacturer of the H-Bridge

Next session I will try some basic line following as well as object detection. I will finish the design of the new IR sensor spacer as well as the mounting system for the Ultrasonic sensor.
