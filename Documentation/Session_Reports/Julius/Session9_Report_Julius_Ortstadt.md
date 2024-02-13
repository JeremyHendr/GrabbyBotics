# Session 9 Report - 13.02.2024 - Julius Ortstadt

## Before today's session
- I made some changes to the stacking system between the Arduino and the H-Bridge.\
![New stacker](/Documentation/Session_Reports/Julius/Pictures/Session_9/Stacker.png)


## During today's session
- I started the print of the stacker. Since it wasn't a very long print, I was able to install it on the robot. Since everything works, this will be the final version.\
![Installed stack](/Documentation/Session_Reports/Julius/Pictures/Session_9/Assembled_Stack.jpg)

- I modelled, in fusion 360, the main power plug for the robot so that I can use it in the 3D model of the robot.\
![Power plug](/Documentation/Session_Reports/Julius/Pictures/Session_9/Power_Plug.png)

- I quickly modelled a spacer needed to position the Nvidia Jetson correctly in the 3D model.\
![Spacer](/Documentation/Session_Reports/Julius/Pictures/Session_9/Spacer.png)

- I inserted the power plug and the fuse holder into the 3D model of the robot and started designing the back panel. This can be subject to change, it is however unlikely, except if we need to make a very big change to the design.\
![Back panel 3D](/Documentation/Session_Reports/Julius/Pictures/Session_9/Back_Panel.png)

- I then cut out the back panel in 5mm CP wood using the laser cuter. I then inserted the plug and the fuse holder and installed everything in the robot.\
![Installed back panel with components](/Documentation/Session_Reports/Julius/Pictures/Session_9/Assembled_Back_Panel.jpg)

- The next step was to solder the cables to the different components and change some of the wiring for easier access and esthetic. This includes shifting the power supply of the Arduino from the front port to the Vin pin and GND.\
![Soldered cables and new power lines for arduino](/Documentation/Session_Reports/Julius/Pictures/Session_9/Soldered_Cables.jpg)

- I also made some adjustments to the 3D model of the robot. These being the new placement of some components (just with more safety margins).\
![New robot render](/Documentation/Session_Reports/Julius/Pictures/Session_9/New_Robot_Render.png)


## For the next session
- I will start thinking about the connector plate as I said last time. This also includes the mecanism that will allow us to "fuse" the movable base and the grabbing system together. We also need to leave some space between the two for the LiDAR system later on in the project.

- I will make a sketch of how the different components still needed will be implemented on the Arduino shield.
