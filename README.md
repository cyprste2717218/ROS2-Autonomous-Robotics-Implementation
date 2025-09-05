<h1>Autonomous Robotic Object Retrieval Solution (AURO-H Module Coursework Assesment)</h1>


<h2>Contents:</h2>

<ul>
	<li><a href="#how-to-run-implementation">How To Run Implementation</a></li> 
	<li><a href="#testable-simulation-scenario">Testable Simulation Scenarios</a></li>
</ul>

<h3 id="how-to-run-implementation">How to Run The Implementation:</h3>

***

The simulation was developed in utilisation of the following software, which must be installed on the host system in order for the simulation to run as intended:

<ul>
	<li><code>ROS2 Humble Hawksbill</code></li>
	<li><code>Gazebo Classic 11</code></li>
	<li><code>rclpy (Python Client library)</code></li>
</ul>

In addition correct simulation operation also requires building a clean workspace including the following packages provided by this repository being placed in the workspaces <code>src</code> directory, as is shown below: (<a href="https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html"> See here for official guidance </a>)


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
In addition, the same process should be repeated for the externally located packages, required in this workspace. These packages are <code>assessment</code>, <code>assessment_interfaces</code>, <code>auro_interfaces</code>, <code>tf_relay</code> and <code>tf_transformations</code>

To find the <code>assessment</code>, <code>assessment_interfaces</code>, <code>auro_interfaces</code> and <code>tf_relay</code> packages you can clone them from the <code>main</code> branch of the <a href="https://github.com/UoY-RoboStar/AURO">AURO</a> repository.

To download and built the <code>tf_transformations</code> package, the process requires using Pythons PIP package installer as follows:

```bash
# In workspace root
$ sudo apt install python3-pip
$ pip3 install transform3d
$ ros2 pkg list | grep tf_transformations

# You should get
# tf_transformations

```
<em>Note: You can pass the '<code>--event-handlers console_direct+</code>' argument to colcon build command to show console output during the build process which may aid debugging in case issues arose. These logs are otherwise found in the '<code>log</code>' dir</em>



In order to access the new built packages, source the overlay in the terminal you wish to use ros2 cli commands relating to these packages and in every other terminal you intend to use for these purposes:

```bash
# Source the overlay
$ source install/local_setup.bash

```
Following these steps, the simulation should be ready for launch with the below testable simulation scenarios!


<h3 id="testable-simulation-scenario">Testable Simulation Scenarios</h3>

***

- Single robot testing scenario with all obstacles enabled and 15mins experiment duration (i.e. using <code>assessment_world.pgm</code> map defined in asssment package):

```bash
$ ros2 launch solution solution_nav2_launch.py num_robots:=1 experiment_duration:=900

```




