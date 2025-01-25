import sys
import os
import yaml


import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, Pose, PoseStamped
from rclpy.executors import ExternalShutdownException
from solution_interfaces.msg import AllowRobotControllerSearch
from std_msgs.msg import Bool
from assessment_interfaces.msg import ItemHolder, ItemHolders
from ament_index_python.packages import get_package_share_directory

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from enum import Enum

class State(Enum):
    SET_INITIAL_GOAL = 0
    SET_MAP_CENTER_GOAL = 1
    NAVIGATING = 2
    
class CurrentNavGoal(Enum):
    INITIAL_GOAL = 0
    CENTER_MAP_GOAL = 1
    WAYPOINTS = 3


class SimpleCommander(Node):

    def __init__(self):
        super().__init__('simple_commander')

        self.state = State.SET_GOAL

        self.current_nav_goal = CurrentNavGoal.INITIAL_GOAL

        # Current colour of item held
        self.current_item_held = ''

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

        # Defining subscriber to /item_holders topic to verify current robot is holding an item

        self.item__holder_subscriber = self.create_subscription(
            ItemHolders,
            'items_holders',
            self.item_holder_callback,
            10
        )


        # Defining publisher for use in controlling navigation state i.e. only robot_controller or simple_commander nodes or both simultaneously

        self.robot_controller_auth_publisher = self.create_publisher(AllowRobotControllerSearch, 'controller_permission', 1)


        # Define initial poses from initial_poses.yaml based on robot namespace

        
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.initial_x
        initial_pose.pose.position.y = self.initial_y

        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        

        # Subscriber callbacks here

        def item_holder_callback(self, msg):

            held_item = msg.data

            if (held_item.robot_id == self.robot_name) & (held_item.holding_item):

                self.current_item_held = held_item.item_colour
                self.state = State.SET_MAP_CENTER_GOAL


        
    def control_loop(self):

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
                center_map_goal_pose.frame_id = 'map'
                center_map_goal_pose.stamp = self.get_clock().now().to_msg()
                center_map_goal_pose.position.x = 0.0  #TO-DO: figure out center of map coordinates
                center_map_goal_pose.position.y = 0.0

                self.navigator.goToPose(center_map_goal_pose)
                self.current_nav_goal = CurrentNavGoal.CENTER_MAP_GOAL
                self.state = State.NAVIGATING
              
            
            case State.NAVIGATING:

                if not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                else:

                    result = self.navigator.getResult()

                    if result == TaskResult.SUCCEEDED:
                        print('Goal succeeded!')
                        #need to check if the goal location was for the center and if so, to send a request to an action server to inform which robot is at the centre - to set this up properly see how the stuff to do with item collection/detection has been setup
                        
                        match self.current_nav_goal: #To-do: move this match statement to function elsewhere for clarity purposes
                            case CurrentNavGoal.INITIAL_GOAL:
                                # permit robot controller to do random search in area navigated to

                                msg = Bool()
                                msg.data = True
                                self.robot_controller_auth_publisher.publish(msg)
                            case CurrentNavGoal.CENTER_MAP_GOAL:
                                print('')
                                # To-do: stuff to determine path with specific waypoints for relevant zone, likely need to create a function to call here,

                                # Return to initial goal location to find another item to pick up and repeat cycle
                                self.state = State.SET_INITIAL_GOAL

                    elif result == TaskResult.CANCELED:
                        print('Goal was canceled!')
                    elif result == TaskResult.FAILED:
                        print('Goal failed!')
                    else:
                        print('Goal has an invalid return status')

            case _:
                pass
    
    def give_robot_controller_auth(self):
        """
        Publishes Bool value True as authorisation for robot controller node to command control of robot.
        """
        msg = Bool()
        msg.data = True
        self.robot_controller_auth_publisher.publish(msg)

    def determine_zone_for_item(self):
        # Interface with item_zone_assignments.yaml config file to determine the assigned zone to deposit item
        print('')

        zone_assignment_path = os.path.join(get_package_share_directory('solution'), 'config', 'item_zone_assignments.yaml')

        with open(zone_assignment_path, 'r') as f:
            zone_assignment_configuration = yaml.safe_load(f)

        #To-Do: need to make nav2 launch file allow for configuration of different assignemtns of zones to item colours as provided in item_zone_assignments.yaml, sticking with hardcoded config of index 1 for now
        assigned_zone = zone_assignment_configuration[1][self.current_item_held]['zone']
        
        return assigned_zone

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
       