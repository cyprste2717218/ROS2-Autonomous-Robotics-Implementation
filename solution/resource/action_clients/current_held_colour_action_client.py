import rclpy 
from rclpy.action import ActionClient
from rclpy.node import Node

from solution_interfaces.action import HeldItem

class HeldColourActionClient(Node):
    def __init__(self):
        super().__init__('held_colour_action_client')
        self._action_client = ActionClient(self, HeldItem, 'heldItem')

    def send_goal(self, data):
        goal_msg = HeldItem.Goal()
        goal_msg.data = data

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)
    
def main (args=None):
    rclpy.init(args=args)

    action_client = HeldColourActionClient()

    future = action_client.send_goal(True)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()