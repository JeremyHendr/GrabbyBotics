# General Information
#### Author: Julius Ortstadt

This code is to be uploaded onto the Arduino that is connected to the Jetson Nano.
It's goal is to:
- Publish the encoder tick data
- Subscribe to the velocity commands sent by the Move Base node of the ROS Navstack
- Convert the velocity commands to a PWM value and add a correction for the robot to move accordingly.

The physical parameters need to be adapted depending on the robot.
To do that, the codes provided in the folder *Code -> Mobile_Base -> Motors* can be used.