# Session 4 Report - 12.12.2023 - Julius Ortstadt

## Before today's session:
- I re-designed the IR sensor spacer in order to correct the mistakes spotted in the previous version, for that I made the design easier and improved the tolerances. While doing this I also cleaned up the sketches in Fusion 360 so that a later modification would be easier. \
![New version of IR sensor spacer](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_4/IR_Spacer.png)


- I made some changes to the frame that will support the robot. This included better access to different components, repositioning of the sled elements so that they better fit and have a better overall distribution. I also added the holes for the screws that will allow the frame to be mounted to the robot.\
![Modified frame](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_4/Frame.png)\
Assembled and rendered, the final frame would look like this:\
![Rendered frame](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_4/Frame_Render.png)

- I finished the design of the ultrasonic sensor case.
![US Sensor case](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_4/US_Sensor_Case.png)

- I modelled some other components that will be needed in the final assembly stages, like this screw spacer for instance.
![Screw spacer](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_4/Screw_Spacer.png)


## During today's session:
- Firstly, before I could start on anything I had to solve the problem of why one wheel was always coming loose. The screws that hold the wheel hub on the motor shaft were shaken loose by vibrations, so I applied some threadlock in order to hold them in place. During later tests, this method seemed to have worked.

- After that I started the print of the new IR spacer and the Ultrasonic sensor case.
After some fit checks, a few adjustments need to be made to the case.

- I then resoldered the cable ends in order to transfer power from the H-bridge to the Arduino so that I would only need one powersupply for the robot.

- I performed some more motor and direction tests since last time I couldn't really make them due to the fact that the wheel kept coming loose.

- I tested the IR sensor to see if it would detect the black line correctly. I then wrote a script to test some basic line following. However, the sensor was to close to the line when fixed to the robot and wouldn't work properly. I tried solving the issue by moving the sensor up and fixing it on the top of the robot after making some holes. However the detection is not working right and I will try to get it to work for the next session. I will also re-do the design of the holding system for this sensor.

Finally, the final assembly:\
![Final assembly](/Documentation/Robo3/Session_Reports/Julius/Pictures/Session_4/Assembly.jpg)


For the next session, as I already said, I will review some designs and adapt them accordingly. I will also continue my research on batteries and charging systems and the overall component layout for the robot in order to fit everything in a robot as small as possible.

