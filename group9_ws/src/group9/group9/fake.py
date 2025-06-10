import rclpy

from std_msgs.msg import Int32
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Fake(Node):

	def __init__(self):
		super().__init__('fake')
		self.publisher1 = self.create_publisher(Int32, '/aruco_number', 10)
		self.publisher2 = self.create_publisher(
			Float32MultiArray, 
			'/pos_array',
			10
		)
		self.publisher2
		self.sent = False
		self.timer = self.create_timer(2, self.timer_callback)
		self.count = 7
		self.pos_array = [[0.0, 0.0], [1.24, -0.244], [2.41, 0.544], [1.76, 1.28], [1.76, 2.42], [1.58, -0.147], [-1.08, 0.689], [-1.87, 2.41]]  
		
		
		
	def timer_callback(self):
		self.get_logger().info('Sending nb of arucos')
		
		msg = Int32()
		msg.data = self.count
		self.publisher1.publish(msg)
		
		if not self.sent:
			msg = Float32MultiArray()
			msg.data = [coord for pose in self.pos_array for coord in pose]
			self.publisher2.publish(msg)
			self.sent = True
	
		
def main(args=None):
	rclpy.init(args=args)
	node = Fake()
	rclpy.spin(node)
	
if __name__ == '__main__':
	main()
		