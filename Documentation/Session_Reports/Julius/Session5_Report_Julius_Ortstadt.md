# Session 5 Report - 09.01.2024 - Julius Ortstadt

## Before today's session:
- After final corrections and fit checks I printed all the Ultrasonic sensor cases so they can me mounted onto the robot when ready.\
![Sensor cases](/Documentation/Session_Reports/Julius/Pictures/Session_5/US_Cases.jpg)

- I saw a design flaw in the frame I made in Fusion 360. This would have led to the frame not being able to be assembled correctly. However, I modified it so that when printed, all the parts can be assembled as intended.\
![Corrected Frame](/Documentation/Session_Reports/Julius/Pictures/Session_5/Corrected_Frame.png)

- I continued working on the line following with the 5 IR sensor array. I also did some research into how this process can be optimized using PID correction etc.

## During today's session:
- After some tests before today's session I came to the conclusion that working with the 5 IR sensor array would be too complicated since there is no way to change the detection distance. I therefore replaced the array with up to 5 individual sensors that can be used in analog mode and which allow changing the detection distance. I then made some changes to the robot in order to implement this for a test phase and then proceeded to test the line following system with 2 sensors.\
![2 sensor line following](/Documentation/Session_Reports/Julius/Pictures/Session_5/2_Sensor_Line_Follow.jpg)

- For a test with 3 sensors I want to make a holding element allowing me to position up to 5 sensors on a rail like system. This would allow for a dynamic change of the distance between each sensor if needed. I started modelling this element in Fusion 360.

- I also did some more research into how all the components will fit in a space as small as possible.

- I continued my research into batteries and charging circuits and how the access ports on the robot (for data, power etc.) will be positioned so that everything is as accessible as possible.


## For next session:
Next session we are holding our mid-project presentation. 
Additionnaly, I will try to get the line following working and start using the motor encoders to get more precise movements and better accuracy.
