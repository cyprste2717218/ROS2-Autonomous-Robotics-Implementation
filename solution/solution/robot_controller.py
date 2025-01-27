#
# Copyright (c) 2024 University of York and others
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
# 
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   * Alan Millard - initial contributor
#   * Pedro Ribeiro - revised implementation
#
 
import sys
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import QoSPresetProfiles
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from assessment_interfaces.msg import Item, ItemList, ZoneList
from auro_interfaces.msg import StringWithPose
from auro_interfaces.srv import ItemRequest
from solution_interfaces.msg import AllowRobotControllerSearch, AllowSimpleCommanderSearch, ItemColourWithRobot, ItemColourWithRobotList
from solution_interfaces.srv import FindItemColour
from ament_index_python.packages import get_package_share_directory


from tf_transformations import euler_from_quaternion
import angles



from enum import Enum
import random
import math


LINEAR_VELOCITY  = 0.3 # Metres per second
ANGULAR_VELOCITY = 0.5 # Radians per second

TURN_LEFT = 1 # Postive angular velocity turns left
TURN_RIGHT = -1 # Negative angular velocity turns right

SCAN_THRESHOLD = 0.9 # Metres per second
 # Array indexes for sensor sectors
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

# Finite state machine (FSM) states
class State(Enum):
    FORWARD = 0
    TURNING = 1
    COLLECTING = 2
    DROPPING_IN_ZONE = 3
    WAITING_TO_RUN = 4


class PreviousState(Enum):
    FORWARD = 0
    TURNING = 1
    COLLECTING = 2
    DROPPING_IN_ZONE = 3
    WAITING_TO_RUN = 4


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        # Class variables used to store persistent values between executions of callbacks and control loop
        self.state = State.WAITING_TO_RUN # Current FSM state
        self.previous_state = self.state
        self.pose = Pose() # Current pose (position and orientation), relative to the odom reference frame
        self.previous_pose = Pose() # Store a snapshot of the pose for comparison against future poses
        self.yaw = 0.0 # Angle the robot is facing (rotation around the Z axis, in radians), relative to the odom reference frame
        self.previous_yaw = 0.0 # Snapshot of the angle for comparison against future angles
        self.turn_angle = 0.0 # Relative angle to turn to in the TURNING state
        self.turn_direction = TURN_LEFT # Direction to turn in the TURNING state
        self.goal_distance = random.uniform(0.3, 0.6) # Goal distance to travel in FORWARD state
        self.scan_triggered = [False] * 4 # Boolean value for each of the 4 LiDAR sensor sectors. True if obstacle detected within SCAN_THRESHOLD
        self.items = ItemList()
        self.current_item_held_colour = ''
        self.in_intended_drop_off_zone = False
        self.detected_zone_colour = ''

        
        self.declare_parameter('robot_id', 'robot1')
        self.robot_id = self.get_parameter('robot_id').value

        # Declaring assigned robot parameters from random assignment in solution_nav2_launch.py 
        self.declare_parameter('assigned_colour', 'blue')
        self.assigned_colour = self.get_parameter('assigned_colour').value
     
        self.declare_parameter('assigned_zone', 'cyan')
        self.assigned_zone = self.get_parameter('assigned_zone').value
     

        # Here we use two callback groups, to ensure that those in 'client_callback_group' can be executed
        # independently from those in 'timer_callback_group'. This allos calling the services below within
        # a callback handled by the timer_callback_group. See https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html
        # for a detailed discussion on the ROS executors and callback groups.
        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.pick_up_service = self.create_client(ItemRequest, '/pick_up_item', callback_group=client_callback_group)
        self.offload_service = self.create_client(ItemRequest, '/offload_item', callback_group=client_callback_group)
        self.find_item_colour_service = self.create_client(FindItemColour, '/find_item_colour')



        self.item_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10, callback_group=timer_callback_group
        )

        # Subscribes to Odometry messages published on /odom topic
        # http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
        #
        # Final argument can either be an integer representing the history depth, or a Quality of Service (QoS) profile
        # https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
        # https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/node.py#L1335-L1338
        # https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/node.py#L1187-L1196
        #
        # If you only specify a history depth, rclpy defaults to QoSHistoryPolicy.KEEP_LAST
        # https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/qos.py#L80-L83
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10, callback_group=timer_callback_group)
        
        # Subscribes to LaserScan messages on the /scan topic
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
        #
        # QoSPresetProfiles.SENSOR_DATA specifices "best effort" reliability and a small queue size
        # https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
        # https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/qos.py#L455
        # https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/qos.py#L428-L431
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value, callback_group=timer_callback_group)


        self.control_permission_subscriber = self.create_subscription(
            Bool,
            'controller_permission',
            self.control_permission_callback,
            1, callback_group=timer_callback_group
        )

        self.zone_sensor_subscriber = self.create_subscription(
            ZoneList,
            'zone',
            self.zone_sensor_callback,
            10, callback_group=timer_callback_group
        )

        self.item_colour_with_robot_subscriber = self.create_subscription(
            ItemColourWithRobotList,
            '/item_colour_with_robot',
            self.item_colour_with_robot_callback,
            10, callback_group=timer_callback_group
        )

        # Publishes Twist messages (linear and angular velocities) on the /cmd_vel topic
        # http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        # 
        # Gazebo ROS differential drive plugin subscribes to these messages, and converts them into left and right wheel speeds
        # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp#L537-L555
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        #self.orientation_publisher = self.create_publisher(Float32, '/orientation', 10)

        # Publishes custom StringWithPose (see auro_interfaces/msg/StringWithPose.msg) messages on the /marker_input topic
        # The week3/rviz_text_marker node subscribes to these messages, and ouputs a Marker message on the /marker_output topic
        # ros2 run week_3 rviz_text_marker
        # This can be visualised in RViz: Add > By topic > /marker_output
        #
        # http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html
        # http://wiki.ros.org/rviz/DisplayTypes/Marker
        self.marker_publisher = self.create_publisher(StringWithPose, 'marker_input', 10, callback_group=timer_callback_group)

        # Defining publisher for use in controlling navigation state i.e. only robot_controller or simple_commander nodes or both simultaneously

        self.simple_commander_auth_publisher = self.create_publisher(Bool, 'commander_permission', 1)

        # Creates a timer that calls the control_loop method repeatedly - each loop represents single iteration of the FSM
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop, callback_group=timer_callback_group)

        # Create Simple Commander Node to utilise nav2 simple commander api - to define initial 2d pose estimate for each robot and direct towards goal 

     

    def item_callback(self, msg):

        self.items.data = msg.data
         

    # Called every time odom_subscriber receives an Odometry message from the /odom topic
    #
    # The Gazebo ROS differential drive plugin generates these messages using kinematic equations, and publishes them
    # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp#L434-L535
    #
    # This plugin is configured with physical measurements of the TurtleBot3 in the SDF file that defines the robot model
    # https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/humble-devel/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf#L476-L507
    #
    # The pose estimates are expressed in a coordinate system relative to the starting pose of the robot
    def odom_callback(self, msg):
        self.pose = msg.pose.pose # Store the pose in a class variable
        self.get_logger().info(f"current pose is: {self.pose}")



        # Uses tf_transformations package to convert orientation from quaternion to Euler angles (RPY = roll, pitch, yaw)
        # https://github.com/DLu/tf_transformations
        #
        # Roll (rotation around X axis) and pitch (rotation around Y axis) are discarded
        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        
        
        self.yaw = yaw # Store the yaw in a class variable

    # Called every time scan_subscriber recieves a LaserScan message from the /scan topic
    #
    # The Gazebo RaySensor calculates distance at which rays intersect with obstacles
    # The data is published by the Gazebo ROS ray sensor plugin
    # https://github.com/gazebosim/gazebo-classic/tree/gazebo11/gazebo/sensors
    # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_ray_sensor.cpp#L178-L205
    #
    # This plugin is configured to match the LiDAR on the TurtleBot3 in the SDF file that defines the robot model
    # http://wiki.ros.org/hls_lfcd_lds_driver
    # https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/humble-devel/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf#L132-L165
    def scan_callback(self, msg):
        # Group scan ranges into 4 segments
        # Front, left, and right segments are each 60 degrees
        # Back segment is 180 degrees


        front_ranges = msg.ranges[331:359] + msg.ranges[0:30] # 30 to 331 degrees (30 to -30 degrees)
        left_ranges  = msg.ranges[31:90] # 31 to 90 degrees (31 to 90 degrees)
        back_ranges  = msg.ranges[91:270] # 91 to 270 degrees (91 to -90 degrees)
        right_ranges = msg.ranges[271:330] # 271 to 330 degrees (-30 to -91 degrees)

        # Store True/False values for each sensor segment, based on whether the nearest detected obstacle is closer than SCAN_THRESHOLD
        self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD


    def control_permission_callback(self, msg):
        print(f"this is the msg receieved on control permission callback {msg}")
        if (msg.data):
            if (self.previous_state == State.COLLECTING):
                print("previous state was collecting so now setting robot to drop in zone state")
                self.previous_state = self.state
                self.state = State.DROPPING_IN_ZONE
            else:
                print("previous state wasn't collecting so heading into forward state for random search for items to pickup")
                self.previous_state = self.state
                self.state = State.FORWARD
        else:
            self.state = State.WAITING_TO_RUN


    def zone_sensor_callback(self, msg):
        zone_data = msg.data

        num_zones = len(zone_data)

        if (num_zones == 1) & (not(self.in_intended_drop_off_zone)):
            detected_zone = zone_data[0]
            
            # determing zone colour based on numbering correspondence in msg type, ie. ZONE_CYAN = 1

            detected_zone_id = detected_zone.zone
            detected_zone_colour = ''

            match detected_zone_id:
                case 1:
                    detected_zone_colour = 'cyan'
                    
                case 2:
                    detected_zone_colour = 'purple'

                case 3:
                    detected_zone_colour = 'green'

                case 4:
                    detected_zone_colour = 'pink'

                case _:
                    pass
            
            self.detected_zone_colour = detected_zone_colour
          

       
    def item_colour_with_robot_callback(self, msg):

        robot_and_colour_list= msg.data

        for item in robot_and_colour_list:
            if (item.robot_id == self.robot_id):
                current_item_colour = item.item_colour
                self.current_item_held_colour = current_item_colour
        
    def call__find_item_colour_service(self, node):
    
        client = self.find_item_colour_service()

        rqt = FindItemColour.Request()
        rqt.request_item_colour = True

        print("about to call the try block")
        try:

            print("in the try block")
            future = client.call_async(rqt)
            rclpy.spin_until_future_complete(node, future)
            response = future.result()
            if response.success:


                self.get_logger().info(f'Current Item colour determined: {self.current_item_held_colour}')
                
            
                # Determining what zone item should go in to compare against current situation, i.e. what zone it is in and which item colour held

                zone_assignment_path = os.path.join(get_package_share_directory('solution'), 'config', 'item_zone_assignments.yaml')

                with open(zone_assignment_path, 'r') as f:
                    zone_assignment_configuration = yaml.safe_load(f)

                assigned_zone = zone_assignment_configuration[1][self.current_item_held_colour]['zone']


                if (assigned_zone == self.detected_zone_colour):
                    print(f"{self.robot_id} is in correct drop off zone {assigned_zone} for item colour {self.current_item_held_colour}")

                    self.in_intended_drop_off_zone = True


                    # Change control logic state, assuming that previously were in a different state to prevent unneccessary re-runs of same part of control loop
                    self.previous_state = self.state

                    if (self.previous_state != State.DROPPING_IN_ZONE):
                        self.state = State.DROPPING_IN_ZONE

                    

            else:
                self.get_logger().info('Unable to determine current item colour held ' + response.message)
        except Exception as e:
            self.get_logger().info('Exception ' + e) 
        else:
            print("Too many zones detected, unsure item can be placed")
            self.in_intended_drop_off_zone = False

    def call_drop_off_service(self, node):
          
          if (self.in_intended_drop_off_zone == True):
            client = self.offload_service()                 
            rqt = ItemRequest.Request()
            rqt.robot_id = self.robot_id
            try:
                future = client.call_async(rqt)
                rclpy.spin_until_future_complete(node, future)
                response = future.result()
                if response.success:

                    self.get_logger().info('Item dropped off.')

                    # saving current state before changing for control coordination purposes between robot_controller and simple_commander nodes
                    self.previous_state = self.state

            
                    # Switching control back to simple_commander node to use nav2 to navigate to center of map
                    self.state = State.WAITING_TO_RUN
                    

                    msg = Bool()
                    msg.data = True
                    self.simple_commander_auth_publisher.publish(msg)

                else:
                    self.get_logger().info('Unable to drop off item: ' + response.message)
            except Exception as e:
                self.get_logger().info('Exception ' + e)   

    def call_pick_up_service(self, node):
        if len(self.items.data) == 0:
            self.previous_pose = self.pose
            self.previous_state = self.state
            self.state = State.FORWARD
            return
        
        item = self.items.data[0]

        client = self.pick_up_service()

        # Obtained by curve fitting from experimental runs.
        estimated_distance = 32.4 * float(item.diameter) ** -0.75 #69.0 * float(item.diameter) ** -0.89

        self.get_logger().info(f'Item Estimated distance {estimated_distance}')

        if estimated_distance <= 0.35:
            rqt = ItemRequest.Request()
            rqt.robot_id = self.robot_id
            try:
                future = client.call_async(rqt)
                rclpy.spin_until_future_complete(node, future)
                response = future.result()
                if response.success:

                    # Switching control back to simple_commander node to use nav2 to navigate to center of map

                    self.get_logger().info('Item picked up.')

                    # saving current state before changing for control coordination purposes between robot_controller and simple_commander nodes
                    self.previous_state = self.state

                    self.state = State.WAITING_TO_RUN
                    self.items.data = []

                    msg = Bool()
                    msg.data = True
                    self.simple_commander_auth_publisher.publish(msg)



                else:
                    self.get_logger().info('Unable to pick up item: ' + response.message)
            except Exception as e:
                self.get_logger().info('Exception ' + e)   

        msg = Twist()
        msg.linear.x = 0.25 * estimated_distance
        msg.angular.z = item.x / 320.0
        self.cmd_vel_publisher.publish(msg)

        



    # Control loop for the FSM - called periodically by self.timer
    def control_loop(self):

        # Send message to rviz_text_marker node
        marker_input = StringWithPose()
        marker_input.text = str(self.state) # Visualise robot state as an RViz marker
        marker_input.pose = self.pose # Set the pose of the RViz marker to track the robot's pose
        self.marker_publisher.publish(marker_input)

        #self.get_logger().info(f'{self.robot_id}: assigned_colour: {self.assigned_colour} assigned_zone: {self.assigned_zone}')

        self.get_logger().info(f"{self.state}")
        
        match self.state:

            case State.FORWARD:
                self.get_logger().info(f"items: {self.items.data}")
                if self.scan_triggered[SCAN_FRONT]:
                    
                    
                    self.previous_yaw = self.yaw
                    self.previous_state = self.state

                    self.state = State.TURNING
                    self.turn_angle = random.uniform(150, 170)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                
                    self.get_logger().info("Detected obstacle in front, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    return
                
                if self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
                    self.previous_yaw = self.yaw
                    self.previous_state = self.state
                    
                    self.state = State.TURNING
                    self.turn_angle = 45

                    if self.scan_triggered[SCAN_LEFT] and self.scan_triggered[SCAN_RIGHT]:
                        self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                        self.get_logger().info("Detected obstacle to both the left and right, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    elif self.scan_triggered[SCAN_LEFT]:
                        self.turn_direction = TURN_RIGHT
                        self.get_logger().info(f"Detected obstacle to the left, turning right by {self.turn_angle} degrees")
                    else: # self.scan_triggered[SCAN_RIGHT]
                        self.turn_direction = TURN_LEFT
                        self.get_logger().info(f"Detected obstacle to the right, turning left by {self.turn_angle} degrees")
                    return
                
                if len(self.items.data) > 0:
                    self.previous_state = self.state
                    self.state = State.COLLECTING
                    return

                msg = Twist()
                msg.linear.x = LINEAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                difference_x = self.pose.position.x - self.previous_pose.position.x
                difference_y = self.pose.position.y - self.previous_pose.position.y
                distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

                # self.get_logger().info(f"Driven {distance_travelled:.2f} out of {self.goal_distance:.2f} metres")

                if distance_travelled >= self.goal_distance:
                    self.previous_yaw = self.yaw
                    self.previous_state = self.state
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(30, 150)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                    self.get_logger().info("Random goal reached, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")

            case State.TURNING:

                self.get_logger().info("Turning state")

                if len(self.items.data) > 0:
                    self.state = State.COLLECTING
                    return

                msg = Twist()
                msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                # self.get_logger().info(f"Turned {math.degrees(math.fabs(yaw_difference)):.2f} out of {self.turn_angle:.2f} degrees")

                yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)                

                if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
                    self.previous_pose = self.pose
                    self.previous_state = self.state
                    self.goal_distance = random.uniform(1.0, 2.0)
                    self.state = State.FORWARD
                    self.get_logger().info(f"Finished turning, driving forward by {self.goal_distance:.2f} metres")

            case State.COLLECTING:
                
                # Calling pick up service
                self.call_pick_up_service()

             

            case State.DROPPING_IN_ZONE:

                #check zone is correct zone for held item type
                #To-do: create action client/server for getting assigned_zone based on item colour, so it passes item_colour (likely should replace logic in simple_commander with this also, create an action client/server for figuring out what item is held)
                self.call__find_item_colour_service()
            
                
                # Checking based on zone sensor callback if we are in correct zone for item being held
                #     
                self.call_drop_off_service()

            case State.WAITING_TO_RUN:
                print("waiting to run state")

            case _:
                pass
        
    def destroy_node(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"Stopping: {msg}")
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()