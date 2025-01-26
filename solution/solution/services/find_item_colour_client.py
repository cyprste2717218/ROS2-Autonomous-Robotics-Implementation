import sys

import rclpy 
from rclpy.node import Node

from solution_interfaces.srv import FindItemColour

class FindItemColourClientAsync(Node):
    def __init__(self):
        super().__init__('find_item_colour_client_async')
        self.cli = self.create_client(FindItemColour, 'find_item_colour')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('find item colour service not available, waiting again...')
        self.req = FindItemColour.Request()

    def send_request(self, request_item_colour):
        self.req.request_item_colour = request_item_colour
    
        return self.cli.call_async(self.req)
    
    
def main ():
    rclpy.init()

    minimal_client = FindItemColourClientAsync()

    future = minimal_client.send_request(str(sys.argv[1]))

    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()

    minimal_client.get_logger().info(
        'Result of request item colour held: %d' % (response.held_item_colour) 
    )

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()