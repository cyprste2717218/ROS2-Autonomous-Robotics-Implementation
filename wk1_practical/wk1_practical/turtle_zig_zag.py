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
		timer_period = 5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		
	def zig(self):
		#twist left and move forward
		msg_1 = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 90.0))
		
		msg_2 = Twist(linear = Vector3(x = 5.0, y = 5.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
		self.publisher.publish(msg_1)
		self.publisher.publish(msg_2)
		
	def zag(self):
		#twist right and move forward
		
		msg_1 = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = -90.0))
		
		msg_2 = Twist(linear = Vector3(x = 0.0, y = 5.0, z = 5.0), angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
		self.publisher.publish(msg_1)
		self.publisher.publish(msg_2)
		
	def rotate_back(self):
		#rotate 180 deg

		msg = Twist(linear = Vector3(x = 0.0, y = 0.0, z = 0.0), angular = Vector3(x = 0.0, y = 0.0, z = 180.0))
		self.publisher.publish(msg)
		
	def timer_callback(self):
		
		while self.i <= 20000:
		
			if self.i % 2 != 0:
				
				self.zig()
			else:
				self.zag()
		
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
