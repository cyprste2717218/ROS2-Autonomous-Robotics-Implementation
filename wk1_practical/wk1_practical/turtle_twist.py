import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class TurtleTwist(Node):

	def __init__(self):
		super().__init__('turtle_twist')
		self.publisher_ = self.create_publisher(
			String,
			'turtle1/cmd_vel',
			10)
		timer_period = 0.5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		
	
		
	def timer_callback(self):
		msg = String()
		msg.data = '{linear: {x: 2.0, y: 0.0, z:0.0}, angular: {x: 0.0, y: 0.0, z:1.8}}'
		self.publisher_.publish(msg)
		self.get_logger().info('Publishing Turtle Twist Command: "%s"' % msg.data)
		self.i += 1 
		
def main(args=None):
	rclpy.init(args=args)
	
	turtle_twist = TurtleTwist()
	
	rclpy.spin(turtle_twist)
	
	turtle_twist.destroy_node()
	rclpy.shutdown()
	
	
if __name__ == '__main__':
	main()
