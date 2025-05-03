import numpy as np
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers #TBC

class ArucoSubscriber(Node):

	def __init__(self):
		super().__init__('aruco_subscriber')
		self.subscription = self.create_subscription(
			ArucoMarkers, 
			'aruco_markers', 
			self.listener_callback,
			10)
		self.subscription
		
	def listener_callback(self, grid):
		self.get_logger().info('Aruco information')
	
def main(args=None):
	rclpy.init(args=args)
	aruco_subscriber = ArucoSubscriber()
	rclpy.spin(aruco_subscriber)
	aruco_subscriber.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()