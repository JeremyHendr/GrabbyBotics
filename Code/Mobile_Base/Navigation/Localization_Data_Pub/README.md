# Information about the package
#### Author: Julius Ortstadt

## Overview
Custom package to convert the pose and goal positions from rviz to understandable messages for the nav_stack package.


## Specifications
- **Development boards:** NVIDIA Jetson Nano 
- **ROS Distribution:** ROS1 Melodic
- **Package Name:** localization_data_pub
- **Node Name:** localization_data_pub
- **Build package and ws:** at root of ws run:
```
catkin_make
```
- **Source ws:** at root of ws run:
``` 
source devel/setup.bash
``` 
- **Used topics:** 
  - Publishes: 
    - */initial_2d* (geometry_msgs/PoseStamped)
    - */goal_2d topic* (geometry_msgs/PoseStamped)
  - Subscribes:
    - */initialpose* (geometry_msgs/PoseWithCovarianceStamped)
    - */move_base_simple/goal* (geometry_msgs/PoseStamped)
- **Launch command:** 
```
roslaunch localization_data_pub localization.launch
```

- **Launch file:**
  - Starts rviz
  - Starts node



## Credit
This package is adapted from the tutorial found [here](https://automaticaddison.com/how-to-create-an-initial-pose-and-goal-publisher-in-ros/).

