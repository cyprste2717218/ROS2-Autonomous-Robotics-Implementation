import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
	launch_include_turtlebot3_world = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('turtlebot3_gazebo'),
				'launch/turtlebot3_world.launch.py'
			)
		)
	)
	
	
	launch_include_rviz2 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('turtlebot3_bringup'),
				'launch/rviz2.launch.py'
			)
		)
	)
	
	collision_avoid_node = Node(
		package='turtlebot3_gazebo',
		executable='turtlebot3_drive',
	)
	
	return LaunchDescription([
		launch_include_turtlebot3_world,
		launch_include_rviz2,
		collision_avoid_node
	])
