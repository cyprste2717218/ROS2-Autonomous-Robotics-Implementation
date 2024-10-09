import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

class TurtleZigZag(Node):
	
	def __init__(self):
		super().__init__('turtle_zig_zag')
		self.publisher = self.create_publisher(
			Twist,
			'/turtle1/cmd_vel',
			10)
		timer_period = 0.5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		
	def zig(self):
		#twist left and move forward

		msg = Twist(linear = Vector3(x = 2.0, y = 10.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 10.0))
		self.publisher.publish(msg)
		
	def zag(self):
		#twist right and move forward

		msg = Twist(linear = Vector3(x = 2.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = -1.8, z = 0.0))
		self.publisher.publish(msg)
		
	def rotate_back(self):
		#rotate 180 deg

		msg = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 180.0, z = 0.0))
		self.publisher.publish(msg)
		
	def timer_callback(self):
		
		while self.i <= 5:
			self.zig()
			self.zag()
			self.get_logger().info('Publishing Turtle Zig Zag Command')
			self.i += 1
		else:
			self.rotate_back()
			self.i = 0
		
		
def main(args=None):
	rclpy.init(args=args)
	
	turtle_zig_zag = TurtleZigZag()
	
	rclpy.spin(turtle_zig_zag)
	
	turtle_zig_zag.destroy_node()
	rclpy.shutdown()
	
	
if __name__ == '__main__':
	main()
