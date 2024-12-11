import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

class TurtleTwist(Node):

	def __init__(self):
		super().__init__('turtle_twist')
		self.publisher = self.create_publisher(
			Twist,
			'/turtle1/cmd_vel',
			10)
		timer_period = 0.5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		
	
		
	def timer_callback(self):
		msg = Twist(linear = Vector3(x = 2.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 1.8))
		self.publisher.publish(msg)
		self.get_logger().info('Publishing Turtle Twist Command')
		self.i += 1 
		
def main(args=None):
	rclpy.init(args=args)
	
	turtle_twist = TurtleTwist()
	
	rclpy.spin(turtle_twist)
	
	turtle_twist.destroy_node()
	rclpy.shutdown()
	
	
if __name__ == '__main__':
	main()
