import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class TopicEcho(Node):

	def __init__(self):
		super().__init__('topic_echo')
		self.subscription = self.create_subscription(
			String,
			'turtle1/cmd_vel',
			self.listener_callback,
			10)
		self.subscription
		
	def listener_callback(self, msg):
		self.get_logger().info('Turtle is alive:' % msg.data)
		
def main(args=None):
	rclpy.init(args=args)
	
	topic_echo = TopicEcho()
	
	rclpy.spin(topic_echo)
	
	topic_echo.destroy_node()
	rclpy.shutdown()
	
	
if __name__ == '__main__':
	main()
