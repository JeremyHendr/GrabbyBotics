# Grabby – Modular Autonomous Mobile Robot for Warehouse Research  
**Authors:** Jeremy Hendrikse, Julius Ortstadt  

## Introduction
This repository contains all software, hardware designs, and documentation related to **Grabby**, an autonomous mobile robot developed as a **proof of concept for warehouse automation and indoor autonomous navigation research**.

The project has been developed over a three-year period as part of the Master’s program in *Robotics and Autonomous Systems* at **Polytech Nice-Sophia**. The primary objective is not direct industrial deployment, but rather the exploration, integration, and evaluation of autonomous navigation techniques on a self-designed robotic platform.

The system is intentionally modular and extensible, serving as a research and experimentation base for mapping, localization, navigation, and manipulation in structured indoor environments.

> The robot architecture is divided into two main subsystems:
> - **Mobile Base**  
>   Responsible for locomotion, power distribution, onboard computation, and autonomous navigation. It hosts the main computer, battery system, sensors, and motor controllers.
> - **Lifting Platform**  
>   A modular upper subsystem designed for package handling. It includes lifting and grabbing mechanisms as well as additional sensors, such as a stereo camera, to assist with perception and task execution.

---

## Software
This section contains all software developed for the robot, covering low-level control, autonomy, and system integration.

The mobile base software stack is built around **ROS 1 (Melodic)** and includes:
- Teleoperation (wireless controller)
- SLAM and mapping
- Localization (AMCL)
- Autonomous navigation
- Sensor fusion (LiDAR and IMU)
- Motor control and odometry

Due to the use of **Ubuntu 24.04 LTS on ARM64**, ROS 1 is executed inside a **Docker container**, ensuring compatibility and reproducibility despite ROS 1 being end-of-life. Detailed configuration steps are provided in the documentation.

The repository also includes:
- Code for Arduino Nano boards
- Parameter tuning files
- Auxiliary scripts and configuration tools

---

## Hardware
The hardware section contains all design files required to reproduce the robot.

Included are:
- 3D models (Fusion 360)
- 2D designs and schematics (Inkscape)
- Mechanical adapters and modular interfaces

The robot features a **modular chassis built from 20×20 mm aluminum profiles**, carbon-reinforced 3D-printed components, and a decentralized electronics architecture. All subsystems are designed for easy assembly, disassembly, and replacement.

---

## Documentation
This section provides comprehensive documentation covering both hardware and software aspects of the project.

It includes:
- Detailed setup instructions for **Ubuntu Server 24.04 LTS** on Raspberry Pi 5
- ROS 1 Melodic installation and configuration using Docker on ARM64
- Mapping and navigation workspace setup
- Sensor configuration and calibration
- Power system design and safety considerations
- Troubleshooting guides and known limitations

All source code, CAD files, and supporting materials are available in this repository to facilitate reproducibility and further development.

---

## Acknowledgements
The development of this project was made possible through the support of **Polytech Nice-Sophia**, which provided the necessary funding, equipment, and academic framework.  
Special thanks are extended to the professors for their guidance in robotics, electronics, sensor fusion, and autonomous systems, which formed the foundation of this work.
