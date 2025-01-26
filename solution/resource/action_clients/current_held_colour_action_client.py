import rclpy 
from rclpy.action import ActionClient
from rclpy.node import Node

from solution_interfaces.action import HeldItem

class HeldColourActionClient(Node):
    def __init__(self):
        super().__init__('held_colour_action_client')
        self._action_client = ActionClient(self, HeldItem, 'heldItem')

    def send_goal(self, item_colour):
        goal_msg = HeldItem.Goal()
        goal_msg.item_colour = item_colour

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    
def main (args=None):
    rclpy.init(args=args)

    action_client = HeldColourActionClient()

    future = action_client.send_goal(True)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()