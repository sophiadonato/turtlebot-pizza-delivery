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
		self.ArucoLocations = np.full(10, np.nan) #How many arucos do we have to consider?
		
	def aruco_callback(self, arucoMarkers):
		self.get_logger().info('Aruco information')
		if len(arucoMarkers.marker_ids) > 0 :
			id = arucoMarkers.marker_ids[0]
			pose = arucoMarkers.poses[0] #Pose in camera_frame
			# How to convert in absolute frame?
			try:
				t = self.tf_buffer.lookup_transform('base_footprint','camera_link',rclpy.time.Time())
			except TransformException as ex:
				self.get_logger().info(f'Could not transform base_footprint to camera_link: {ex}')
				return
			self.ArucoLocations[id] = do_transform_pose(pose,t)
		if not np.isnan(self.ArucoLocations).any():
			self.get_logger().info('All arucos have been visited')
	
def main(args=None):
	rclpy.init(args=args)
	aruco_subscriber = ArucoSubscriber()
	rclpy.spin(aruco_subscriber)
	aruco_subscriber.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()