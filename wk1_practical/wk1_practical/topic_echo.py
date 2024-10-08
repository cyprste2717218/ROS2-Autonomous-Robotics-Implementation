import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TopicEcho(Node):

	def __init__(self):
		super().__init__('topic_echo')
		self.subscription = self.create_subscription(
			Twist,
			'/turtle1/cmd_vel',
			self.listener_callback,
			10)
		self.subscription
		
	def listener_callback(self, msg):
		self.get_logger().info(f"Received: Twist - linear: ({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), angular: ({msg.angular.x}, {msg.angular.y}, {msg.angular.z})")
		
def main(args=None):
	rclpy.init(args=args)
	
	topic_echo = TopicEcho()
	
	rclpy.spin(topic_echo)
	
	topic_echo.destroy_node()
	rclpy.shutdown()
	
	
if __name__ == '__main__':
	main()
