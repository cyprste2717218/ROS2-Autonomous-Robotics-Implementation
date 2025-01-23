Autonomous Robotic Object Retrieval Solution (AURO-H Module Coursework Assesment)


Contents:

- How To Run Implementation
- Testable Simulation Scenarios


How to Run The Implementation:
-------------------------------------------------

mention about needing tf_transformations package, sudo apt installing python3-pip and pip3 install transform3d

This simulation utilises SLAM via Cartographer, thus the following instructions must be followed to install the needed packages:



1). Install the Cartographer packages and verify presence within the ROS2 system using the following bash commands:


```bash
$ sudo apt install ros-humble-cartographer
$ ros2 pkg list | grep cartographer

# You should get
# cartographer_ros
# cartographer_ros_msgs
# turtlebot3_cartographer

```
The terminal output as shown above should evidence the presence of the cartographer_ros, cartographer_ros_msgs and turtlebot3_cartographer packages.

2). Rebuild the workspace with the new pkgs and source the local setup file:

Within the workspace directory, execute the following commands

```bash
$ colcon build 
$ source install/local_setup.bash
```



Testable Simulation Scenarios


- 3 robots enabled (alongside all defaults)


(#1) ros2 launch solution solution_launch.py num_robots:=3