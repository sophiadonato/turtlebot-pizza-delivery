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
		self.timer = self.create_timer(2, self.timer_callback)
		self.count = 7
		self.pos_array = [[0.0, 0.0], [1.9, 0.947], [0.82, -2.09], [3.6, 0.037]]  
		
		
		
	def timer_callback(self):
		self.get_logger().info('Sending nb of arucos')
		
		msg = Int32()
		msg.data = self.count
		self.publisher1.publish(msg)
		
		msg = Float32MultiArray()
		msg.data = [coord for pose in self.pos_array for coord in pose]
		self.publisher2.publish(msg)
	
		
def main(args=None):
	rclpy.init(args=args)
	node = Fake()
	rclpy.spin(node)
	
if __name__ == '__main__':
	main()
		