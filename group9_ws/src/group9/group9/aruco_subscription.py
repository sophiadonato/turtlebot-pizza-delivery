import numpy as np
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers #TBC
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Pose, PoseArray,TransformStamped
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformException
import KalmanFilter

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
		self.publisher1 = self.create_publisher(PoseArray, 'arucos_location', 10)
		self.publisher2 = self.create_publisher(PoseArray, 'robot_location', 10)
		self.ArucoLocations = [None]*nbArucos  #How many arucos do we have to consider?

		#We implement a Kalman filter to get rid of camera noise
		self.filters = {}
		
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
			pose = do_transform_pose(pose,t)
			if id not in self.filters:
				self.filters[id] = KalmanFilter()
				
			self.filters[id].update(pose) #KF on the global aruco location
			self.ArucoLocations[id] = self.filters[id].get_estimate()
   
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

def PoseToVisitForGivenAruco(ArucoPose): #To be improved
		#What is the orientation in the ArucoPose ?
		#Values have been found using simulations on Gazebo
		t1 = TransformStamped()
  
		t1.transform.translation.x = 0.5
		t1.transform.translation.y = 0.0
		t1.transform.translation.z = 0.0

		# We want to get a vector that face the wall
		# We have rotation of the axis of 180 degrees around z axis
		# And rotation around y of -90 degrees
		t1.transform.rotation.x = 0.0
		t1.transform.rotation.y = 0.0
		t1.transform.rotation.z = 1.0
		t1.transform.rotation.w = 0.0 
  
		t2 = TransformStamped()

		t2.transform.translation.x = 0.0
		t2.transform.translation.y = 0.0
		t1.transform.translation.z = 0.0

		# We want to get a vector that face the wall
		# And rotation around y of -90 degrees ? How come? TBC
		t1.transform.rotation.x = 0.0
		t1.transform.rotation.y = -0.707
		t1.transform.rotation.z = 0.0
		t1.transform.rotation.w = 0.707 
		
		return do_transform_pose(do_transform_pose(ArucoPose, t1),t2)



	
def main(args=None):
	rclpy.init(args=args)
	aruco_subscriber = ArucoSubscriber()
	rclpy.spin(aruco_subscriber)
	aruco_subscriber.destroy_node() #Check that it's needed or not if node is destroyed in aruco_callback
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()