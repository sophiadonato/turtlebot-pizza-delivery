import numpy as np
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers #TBC
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformException

class ArucoSubscriber(Node):

	def __init__(self):
		super().__init__('aruco_subscriber')
		self.subscription = self.create_subscription(
			ArucoMarkers, 
			'aruco_markers', 
			self.aruco_callback,
			10)
		self.subscription
		self.tf_buffer = Buffer()
		self.listener = TransformListener(self.tf_buffer, self)
		self.ArucoLocations = [None]*10 #How many arucos do we have to consider?
		
	def aruco_callback(self, arucoMarkers):
		if len(arucoMarkers.marker_ids) > 0 :
			id = arucoMarkers.marker_ids[0]
			self.get_logger().info(f'Aruco information : ID : {id}')
			pose = arucoMarkers.poses[0] #Pose in camera_frame
			# How to convert in absolute frame?
			try:
				t = self.tf_buffer.lookup_transform('map','camera_optical_joint',rclpy.time.Time())
			except TransformException as ex:
				self.get_logger().info(f'Could not transform map to camera_optical_joint: {ex}')
				return
			self.ArucoLocations[id] = do_transform_pose(pose,t)
			self.get_logger().info(f'{self.ArucoLocations[id].position.x}')

		if all(pose is not None for pose in self.ArucoLocations):
			self.get_logger().info('All arucos have been visited')
	
def main(args=None):
	rclpy.init(args=args)
	aruco_subscriber = ArucoSubscriber()
	rclpy.spin(aruco_subscriber)
	aruco_subscriber.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()