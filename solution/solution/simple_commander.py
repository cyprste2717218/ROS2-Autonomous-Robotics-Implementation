import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, Pose, PoseStamped
from rclpy.executors import ExternalShutdownException
from solution_interfaces.msg import AllowRobotControllerSearch
from std_msgs.msg import Bool
from assessment_interfaces.msg import ItemHolder, ItemHolders

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from enum import Enum

class State(Enum):
    SET_GOAL = 0
    SET_MAP_CENTER_GOAL = 1
    NAVIGATING = 2
    

class SimpleCommander(Node):

    def __init__(self):
        super().__init__('simple_commander')

        self.state = State.SET_GOAL

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

            info = msg.data

            if (info.robot_id == self.robot_name):
                self.state = State.SET_MAP_CENTER_GOAL


        
    def control_loop(self):

        match self.state:

            case State.SET_GOAL:
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = self.initial_goal_x
                goal_pose.pose.position.y = self.initial_goal_y

                self.navigator.goToPose(goal_pose)
            
            case State.SET_MAP_CENTER_GOAL:
                center_map_goal_pose = PoseStamped()
            
            case State.NAVIGATING:

                if not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                else:

                    result = self.navigator.getResult()

                    if result == TaskResult.SUCCEEDED:
                        print('Goal succeeded!')
                        #need to check if the goal location was for the center and if so, to send a request to an action server to inform which robot is at the centre - to set this up properly see how the stuff to do with item collection/detection has been setup
                        
                        # permit robot controller to do random search in area navigated to

                        msg = Bool()
                        msg.data = True
                        self.robot_controller_auth_publisher.publish(msg)

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
       