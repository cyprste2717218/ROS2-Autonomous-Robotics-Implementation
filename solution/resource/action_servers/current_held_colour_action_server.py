import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from solution_interfaces.action import HeldItem

class HeldColourActionServer(Node):

    def __init__(self):
        super().__init__('held_colour_action_server')
        self.action_server = ActionServer(
            self,
            HeldItem,
            'heldItem',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing current item colour held request goal')
        result = HeldItem.Result()
        return result
    
def main(args=None):
    rclpy.init(args=args)

    held_colour_action_server = HeldColourActionServer()

    rclpy.spin(held_colour_action_server)

if __name__ == '__main__':
    main()