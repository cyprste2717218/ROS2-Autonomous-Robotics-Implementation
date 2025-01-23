import os
import yaml
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, Shutdown, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap, PushRosNamespace, RosTimer

def robot_controller_actions(context : LaunchContext):

    num_robots = 1
        
    yaml_path = os.path.join(get_package_share_directory('solution'), 'config', 'cartographer_config', 'map_creation_robot_pose.yaml')

    with open(yaml_path, 'r') as f:
        configuration = yaml.safe_load(f)

    initial_poses = configuration[1]

    actions = []

    for robot_number in range(1, num_robots + 1):

        robot_name = 'robot' + str(robot_number)

        group = GroupAction([

            PushRosNamespace(robot_name),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),

            Node(
                package='solution',
                executable='robot_controller',
                # prefix=['xfce4-terminal --tab --execute'], # Opens in new tab
                # prefix=['xfce4-terminal --execute'], # Opens in new window
                # prefix=['gnome-terminal --tab --execute'], # Opens in new tab
                # prefix=['gnome-terminal --window --execute'], # Opens in new window
                # prefix=['wt.exe --window 0 new-tab wsl.exe -e bash -ic'], # Opens in new tab
                # prefix=['wt.exe wsl.exe -e bash -ic'], # Opens in new window
                output='screen',
                parameters=[initial_poses[robot_name]]),

            # Node(
            #     package='turtlebot3_gazebo',
            #     executable='turtlebot3_drive',
            #     output='screen'),

        ])

        actions.append(group)

    return actions


def generate_launch_description():

    package_name = 'solution'

    # Positioning function pack
    pkg_share = '/home/auro/shared/Documents/AURO/auro_ws/src/solution'
    
    # Configure node launch information 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Map resolution
    resolution = LaunchConfiguration('resolution', default='0.05')
    # Map publish period  
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # Configuration file folder path
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config/cartographer_config') )
    # Configuration file
    configuration_basename = LaunchConfiguration('configuration_basename', default='map_builder_2d.lua')
    
    random_seed = LaunchConfiguration('random_seed')


    #Nodes
    """ cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
 """
    


    rviz_config = PathJoinSubstitution([FindPackageShare('assessment'), 'rviz', 'namespaced.rviz'])
    rviz_windows = PathJoinSubstitution([FindPackageShare('assessment'), 'config', 'rviz_windows.yaml'])

    cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_cartographer'),
                'launch',
                'cartographer.launch.py'
                ])
        ),
        launch_arguments={'use_sim_time': 'true',
                          'configuration_basename': 'config/cartographer_config/map_builder_2d.lua'
                          }.items()
    )


    robot_controller_cmd = OpaqueFunction(function=robot_controller_actions)

    assessment_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('assessment'),
                'launch',
                'assessment_launch.py'
                ])
        ),
        launch_arguments={'num_robots': '1',
                          'visualise_sensors': 'false',
                          'odometry_source': 'ENCODER',
                          'sensor_noise': 'false',
                          'use_rviz': 'false',
                          'rviz_config': rviz_config,
                          'rviz_windows': rviz_windows,
                          'obstacles': 'true',
                          'item_manager': 'false',
                          'use_nav2': 'true',
                          'headless': 'false',
                          'limit_real_time_factor': 'true',
                          'wait_for_items': 'false',
                          'initial_pose_package': 'solution',
                          'inital_pose_file': 'config/cartographer_config/map_creation_robot_pose.yaml'
                          # 'extra_gazebo_args': '--verbose',
                          }.items()
    )

    #Launch file
    ld = LaunchDescription()

    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(assessment_cmd)
    ld.add_action(cartographer_cmd)
    ld.add_action(robot_controller_cmd)

    return ld

    """    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node) """
