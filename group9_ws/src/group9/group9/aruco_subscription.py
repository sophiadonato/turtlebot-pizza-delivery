import numpy as np
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers #TBC
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Pose, PoseArray,TransformStamped
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
		nbArucos = 2
		self.publisher1 = self.create_publisher(PoseArray, 'arucos_location', 10) #Pose_Array does exist, it should be tried
		self.publisher2 = self.create_publisher(PoseArray, 'robot_location', 10)
		self.ArucoLocations = [None]*nbArucos  #How many arucos do we have to consider?
		
	def aruco_callback(self, arucoMarkers):
		if len(arucoMarkers.marker_ids) > 0 :
			id = arucoMarkers.marker_ids[0]
			self.get_logger().info(f'Aruco information : ID : {id}')
			pose = arucoMarkers.poses[0]
			try:
				t = self.tf_buffer.lookup_transform('map','camera_optical_joint',rclpy.time.Time())
			except TransformException as ex:
				self.get_logger().info(f'Could not transform map to camera_optical_joint: {ex}')
				return
			self.ArucoLocations[id] = do_transform_pose(pose,t)
   
		if all(pose is not None for pose in self.ArucoLocations):
			self.get_logger().info('All arucos have been visited')
			self.aruco_locations_publisher()
			self.destroy_node()
			rclpy.shutdown()

	def aruco_locations_publisher(self):
		#Do we need to publish other informations?
		#See aruco_node to see how it's done (Using ArrayPose())
		locationArucos = PoseArray()
		locationArucos.header.frame_id = "Poses where the arucos are located"
		robotToVisit = PoseArray()
		robotToVisit.header.frame_id = "Poses where the robot should go to visit the aruco"
		for Pose in self.ArucoLocations:
			locationArucos.poses.append(Pose)
			robotToVisit.poses.append(PoseToVisitForGivenAruco(Pose))
		self.publisher1.publish(locationArucos)
		self.publisher2.publish(robotToVisit)

def PoseToVisitForGivenAruco(ArucoPose):
		#What is the orientation in the ArucoPose ?
		t1 = TransformStamped() #UpperLeftCornerTransform

		# This transform represents a shift from (0, 0) to (0, 30)
		t1.transform.translation.x = 0.0
		t1.transform.translation.y = 0.30
		t1.transform.translation.z = 0.0

		# Rotation of the Pose should be confirmed using TurtleBot
		# We want to get a vector that face the wall
		t1.transform.rotation.x = 0.0
		t1.transform.rotation.y = 0.0
		t1.transform.rotation.z = 3.1415 
		t1.transform.rotation.w = 1.0 #What is w?
		
		return do_transform_pose(ArucoPose, t1)

	
def main(args=None):
	rclpy.init(args=args)
	aruco_subscriber = ArucoSubscriber()
	rclpy.spin(aruco_subscriber)
	aruco_subscriber.destroy_node() #Check that it's needed or not if node is destroyed in aruco_callback
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()