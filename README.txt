Autonomous Robotic Object Retrieval Solution (AURO-H Module Coursework Assesment)


Contents:

- How To Run Implementation
- Testable Simulation Scenarios


How to Run The Implementation:
-------------------------------------------------

The simulation was developed in utilisation of the following software, which must be installed on the host system in order for the simulation to run as intended:

- ROS2 Humble Hawksbill
- Gazebo Classic 11
- rclpy (Python Client library)


In addition correct simulation operation also requires building a clean workspace including the following packages provided by this repository being placed in the workspaces 'src/' directory, as is shown below:

(see here for official guidance: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html),


```bash
# Go to workspace root
$ cd ~/ros2_ws
# Build workspace 
$ colcon build 
# 
$ ros2 pkg list | grep solution

# You should get
# solution
# solution_interfaces

```
In addition, the same process should be repeated for the externally located packages, required in this workspace. These packages are 'assessment', 'assessment_interfaces', 'auro_interfaces', 'tf_relay' and 'tf_transformations'

To find the 'assessment', 'assessment_interfaces', 'auro_interfaces'  and 'tf_relay' packages you can clone them from the 'main' branch of the 'AURO' repository (link: https://github.com/UoY-RoboStar/AURO)

To download and built the 'tf_transformations' package, the process requires using Pythons PIP package installer as follows:

```bash
# In workspace root
$ sudo apt install python3-pip
$ pip3 install transform3d
$ ros2 pkg list | grep tf_transformations

# You should get
# tf_transformations

```
Note: You can pass the '--event-handlers console_direct+' argument to colcon build command to show console output during the build process which may aid debugging in case issues arose. These logs are otherwise found in the 'log' dir



In order to access the new built packages, source the overlay in the terminal you wish to use ros2 cli commands relating to these packages and in every other terminal you intend to use for these purposes:

```bash
# Source the overlay
$ source install/local_setup.bash

```
Following these steps, the simulation should be ready for launch with the below testable simulation scenarios!


Testable Simulation Scenarios

- Single robot testing scenario with all obstacles enabled and 20mins experiment duration (i.e. using assessment_world.pgm map defined in asssment package):

```bash
$ ros2 launch solution solution_nav2_launch.py num_robots:=1 experiment_duration=1200

```




