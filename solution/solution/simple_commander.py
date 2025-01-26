import sys
import os
import yaml

import copy
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, Pose, PoseStamped
from rclpy.executors import ExternalShutdownException
from solution_interfaces.msg import AllowRobotControllerSearch, AllowSimpleCommanderSearch
from std_msgs.msg import Bool
from assessment_interfaces.msg import ItemHolder, ItemHolders
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from enum import Enum

class State(Enum):
    SET_INITIAL_GOAL = 0
    SET_MAP_CENTER_GOAL = 1
    SET_WAYPOINTS = 2
    NAVIGATING = 3
    IDLE = 4

class CurrentNavGoal(Enum):
    INITIAL_GOAL = 0
    CENTER_MAP_GOAL = 1
    WAYPOINTS = 3


class PreviousNavGoal(Enum):
    INITIAL_GOAL = 0
    CENTER_MAP_GOAL = 1
    WAYPOINTS = 3

class SimpleCommander(Node):

    def __init__(self):
        super().__init__('simple_commander')

        self.state = State.SET_INITIAL_GOAL
       

        self.current_nav_goal = CurrentNavGoal.INITIAL_GOAL
        self.previous_nav_goal = self.current_nav_goal

        # Current colour of item held
        self.current_item_held = ''
        self.already_holding_item = False
        
        self.waypoints = []

        self.navigator = BasicNavigator()

        # Defining params for initial location, first goal point to reach and robot_name
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').value

        self.declare_parameter('initial_loc_x', 0.0)
        self.initial_x = self.get_parameter('initial_loc_x').value

        self.declare_parameter('initial_loc_y', 0.0)
        self.initial_y = self.get_parameter('initial_loc_y').value

        self.declare_parameter('initial_loc_yaw', 0.0)
        self.initial_yaw = self.get_parameter('initial_loc_yaw').value


        self.declare_parameter('initial_goal_x', 0.0)
        self.initial_goal_x = self.get_parameter('initial_goal_x').value

        self.declare_parameter('initial_goal_y', 0.0)
        self.initial_goal_y = self.get_parameter('initial_goal_y').value

        self.declare_parameter('initial_goal_yaw', 0.0)
        self.initial_goal_yaw = self.get_parameter('initial_goal_yaw').value


        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()


        # Defining subscriber to /item_holders topic to verify current robot is holding an item

        self.item__holder_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.item_holder_callback,
            10, callback_group=timer_callback_group
        )

        # Defining subscriber to /commander_permission topic to determine permission for commander node to resume control

        self.simple_commander_auth_subscriber = self.create_subscription(
            Bool,
            'commander_permission',
            self.commander_permission_callback,
            1, callback_group=timer_callback_group
        )


        # Defining publisher for use in controlling navigation state i.e. only robot_controller or simple_commander nodes or both simultaneously

        self.robot_controller_auth_publisher = self.create_publisher(Bool, 'controller_permission', 1)


        # Define initial poses from initial_poses.yaml based on robot namespace

        
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.initial_x
        initial_pose.pose.position.y = self.initial_y

        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()

        self.timer_period = 3.0
        self.timer = self.create_timer(self.timer_period, self.control_loop, callback_group=timer_callback_group)
        

        # Subscriber callbacks here

    def item_holder_callback(self, msg):
        #print("item holder callback not being called")
        held_item = msg.data[-1]
        print(f"this is the held_item {held_item}")
        if (held_item.robot_id == self.robot_name) & (held_item.holding_item) & (not(self.already_holding_item)):

            self.already_holding_item = True

            current_item_held = held_item.item_colour
            parsed_current_item_held = current_item_held.lower()
            self.current_item_held = parsed_current_item_held

            print(f"currently held item colour is: {self.current_item_held}")
            self.state = State.SET_MAP_CENTER_GOAL
        if held_item.holding_item == False:
            self.already_holding_item = False

    def commander_permission_callback(self, msg):
            if (msg.data) & (self.previous_nav_goal == CurrentNavGoal.INITIAL_GOAL):
                self.state = State.SET_MAP_CENTER_GOAL

            elif (msg.data) & (self.previous_nav_goal == CurrentNavGoal.WAYPOINTS):
                # reversing waypoints to route back to center of map after item deposit
                reversed_waypoints = list(reversed(self.waypoints))
                self.waypoints = reversed_waypoints
                self.state = State.SET_WAYPOINTS

            
        
    def control_loop(self):
        
        print(f"Simple Commander Node State: {self.state} Current Nav Goal: {self.current_nav_goal}")

        match self.state:

            case State.SET_INITIAL_GOAL:
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = self.initial_goal_x
                goal_pose.pose.position.y = self.initial_goal_y

                self.navigator.goToPose(goal_pose)
                self.state = State.NAVIGATING
            
            case State.SET_MAP_CENTER_GOAL:
                center_map_goal_pose = PoseStamped()
                center_map_goal_pose.header.frame_id = 'map'
                center_map_goal_pose.header.stamp = self.get_clock().now().to_msg()
                center_map_goal_pose.pose.position.x = 0.0  
                center_map_goal_pose.pose.position.y = 0.0

                self.navigator.goToPose(center_map_goal_pose)
                self.previous_nav_goal = self.current_nav_goal
                self.current_nav_goal = CurrentNavGoal.CENTER_MAP_GOAL

                self.state = State.NAVIGATING
            
            case State.SET_WAYPOINTS:
                # To-Do: implement nav_start if can get below logic for cancelling task for robot to work
                # nav_start = self.navigator.get_clock().now()
                print("following waypoints")
                self.navigator.followWaypoints(self.waypoints)
                self.previous_nav_goal = self.current_nav_goal
                self.current_nav_goal = CurrentNavGoal.WAYPOINTS
                self.state = State.NAVIGATING
            
            case State.NAVIGATING:
                feedback = self.navigator.getFeedback()

                if not self.navigator.isTaskComplete():
                    
                    
                    # Logging current waypoint if waypoints are being followed
                    if (len(self.waypoints) > 0):
                        self.navigator.get_logger().info('Executing current waypoint: ' + str(feedback.current_waypoint + 1) + '/' + str(len(self.waypoints)))
                        
                        # Cancelling task if robot gets stuck after max period time (need to refactor most likely to make this work properly with rest of control logic)

                        """ now = self.navigator.get_clock().now()

                        if now - nav_start > Duration(seconds=500.0):
                            self.navigator.cancelTask() """
                else:

                    result = self.navigator.getResult()

                    if result == TaskResult.SUCCEEDED:
                        print('Goal succeeded!')
                        #need to check if the goal location was for the center and if so, to send a request to an action server to inform which robot is at the centre - to set this up properly see how the stuff to do with item collection/detection has been setup
                        
                        

                        match self.current_nav_goal: #To-do: move this match statement to function elsewhere for clarity purposes
                            case CurrentNavGoal.INITIAL_GOAL:
                                
                                # Cancelling task to prevent reoccurring success messages when control loop is called
                                self.navigator.cancelTask()

                                # permit robot controller to do random search in area navigated to

                                msg = Bool()
                                msg.data = True
                                self.robot_controller_auth_publisher.publish(msg)

                                # Temporarily switch to IDLE State while robot controller does random search to find item to pick up

                                self.state = State.IDLE

                            case CurrentNavGoal.CENTER_MAP_GOAL:

                                # Cancelling task to prevent reoccurring success messages when control loop is called
                                self.navigator.cancelTask()
                                
                                print('before getting assigned zone')
                                assigned_zone = self.determine_zone_for_item()
                                print('aftergetting assigned zone')

                                self.determine_waypoints_to_zone(assigned_zone)
                               
                                self.state = State.SET_WAYPOINTS

                            case CurrentNavGoal.WAYPOINTS:

                                print(f"Travelled {feedback.current_waypoint + 1} / 3 waypoints ")

                                if (feedback.current_waypoint + 1) == len(self.waypoints):
                                    

                                    print("All waypoints navigated")
                                    # Cancelling task to prevent reoccurring success messages when control loop is called
                                    self.navigator.cancelTask()

                                    # Giving back control to robot controller to deposit item in zone, halting simple commander node operations temporarily
                                    msg = Bool()
                                    msg.data = True
                                    self.robot_controller_auth_publisher.publish(msg)

                                    self.State = State.IDLE
                               
                                  

                                    

                    elif result == TaskResult.CANCELED:
                        print('Goal was canceled!')
                    elif result == TaskResult.FAILED:
                        print('Goal failed!')
                    else:
                        print('Goal has an invalid return status')

            case State.IDLE:
                print('Simple commander currently in IDLE state')

            case _:
                pass
    
    def determine_zone_for_item(self):
        # Interface with item_zone_assignments.yaml config file to determine the assigned zone to deposit item
        print("determine zone for items func is actually called")      

        zone_assignment_path = os.path.join(get_package_share_directory('solution'), 'config', 'item_zone_assignments.yaml')

        with open(zone_assignment_path, 'r') as f:
            zone_assignment_configuration = yaml.safe_load(f)

        print("gets to here in determine zone for items func")
        print(f"these are the params passed: {self.current_item_held}")
        #To-Do: need to make nav2 launch file allow for configuration of different assignemtns of zones to item colours as provided in item_zone_assignments.yaml, sticking with hardcoded config of index 1 for now
        assigned_zone = zone_assignment_configuration[1][self.current_item_held]['zone']

        
        print(f"Assigned zone for item of colour {self.current_item_held} is {assigned_zone}")

        return assigned_zone
    
    # To-do: ensure setting up function with both self and current_assinged_zone works and is a valid setup, am unsure if calling it with just current_assigned_zone will work as is currently what is being done
    def determine_waypoints_to_zone(self, current_assigned_zone):
        nav_to_zone_waypoints_path = os.path.join(get_package_share_directory('solution'), 'config', 'nav_to_zone_waypoints.yaml')

        with open(nav_to_zone_waypoints_path, 'r') as f:
            nav_to_zone_waypoints_configuration = yaml.safe_load(f)

        nav_waypoints = nav_to_zone_waypoints_configuration[current_assigned_zone]

        print(f'Current nav to zone waypoint config is here: {current_assigned_zone}')
        waypoint_1 = nav_waypoints['waypoint_1']
        waypoint_2 = nav_waypoints['waypoint_2']
        waypoint_3 = nav_waypoints['waypoint_3']

       
        raw_waypoint_poses = [waypoint_1, waypoint_2, waypoint_3]
        print(f'this is the raw waypoint poses: {raw_waypoint_poses}')
       

        waypoint_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        for wp in raw_waypoint_poses:
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            waypoint_poses.append(copy.deepcopy(pose))
        
        self.waypoints = waypoint_poses

    def destroy_node(self):
        self.get_logger().info(f"Shutting down Simple Commander node")
        self.navigator.lifecycleShutdown()
        super().destroy_node()

def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = SimpleCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
       