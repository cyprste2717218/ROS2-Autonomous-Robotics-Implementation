import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, Pose, PoseStamped
from rclpy.executors import ExternalShutdownException

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class SimpleCommander(Node):

    def __init__(self):
        super().__init__('simple_commander')

        self.navigator = BasicNavigator()


        self.declare_parameter('initial_x', 0.0)
        self.initial_x = self.get_parameter('initial_x').value

        self.declare_parameter('initial_y', 0.0)
        self.initial_y = self.get_parameter('initial_y').value

        self.declare_parameter('initial_yaw', 0.0)
        self.initial_yaw = self.get_parameter('initial_yaw').value

        # Define initial poses from initial_poses.yaml based on robot namespace
        # print(f"initial_x is type: {type(self.initial_x)} {self.initial_x} initial_y is type: {type(self.initial_y)} {self.initial_y} initial_yaw is type: {type(self.initial_yaw)} {self.initial_yaw} ")
        
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.initial_x
        initial_pose.pose.position.y = self.initial_y

        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()
        
        



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
       