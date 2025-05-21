import numpy as np
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers #TBC
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Pose, PoseArray,TransformStamped
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class KalmanFilter:
    def __init__(self):
        self.x = np.zeros((3,1))
        self.q = np.zeros(4)
        self.sigma = np.eye(3)*10.0 #Initial uncertainty (map of 10m?)

    def updatePosition(self, yt):
        yt = yt.reshape(3,1)
        #Motion model
        # x_{t+1} = x_t + v_t

        #Sensor model
        #y_t = C x_t + w
        A = np.eye(3)
        C = np.eye(3)
        R = np.eye(3)*0.0001 #Process noise (very low)
        Q = np.eye(3)*0.05 #Measurement noise
        mu_bar = A@self.x.reshape(3,1)
        sigma_bar = A@self.sigma@A.T + R

        K = sigma_bar@C.T@np.linalg.inv(C@sigma_bar@C.T + Q)
        self.x = (mu_bar + K@(yt - C@mu_bar))
        self.sigma = (np.eye(3)- K@C)@sigma_bar
    
    def updateRotation(self, qt, alpha = 0.05):
        # Rotations are not linear => need a EKF, for simplicity, we will consider weighted average for updating
        # The camera is better in orientation than in position
        # First intuition is therefore to only average update the quaternions
        q = (1- alpha)*self.q + alpha*qt
        self.q = q/np.linalg.norm(q)
        
    
    def updateWithPose(self,pose):
        yt = np.array([pose.position.x, pose.position.y,pose.position.z])
        qt = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w])
        self.updatePosition(yt)
        self.updateRotation(qt)
    
    def get_estimate(self):
        p = Pose()
        x = self.x.flatten()
        p.position.x = x[0]
        p.position.y = x[1]
        p.position.z = x[2]
        
        p.orientation.x = self.q[0]
        p.orientation.y = self.q[1]
        p.orientation.z = self.q[2]
        p.orientation.w = self.q[3]
        return p


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
		nbArucos = 10
		self.publisher1 = self.create_publisher(PoseArray, 'arucos_location', 10)
		self.publisher2 = self.create_publisher(PoseArray, 'robot_location', 10)
		self.ArucoLocations = [None]*nbArucos  #How many arucos do we have to consider?

		#We implement a Kalman filter to get rid of camera noise
		self.filters = {}
		
		#Broadcaster
		self.tf_static_broadcaster = StaticTransformBroadcaster(self)

	def aruco_callback(self, arucoMarkers):
		for i in range(0, len(arucoMarkers.marker_ids)):
			id = arucoMarkers.marker_ids[i]
			self.get_logger().info(f'Aruco information : ID : {id}')
			pose = arucoMarkers.poses[i]
			try:
				t = self.tf_buffer.lookup_transform('map','camera_optical_joint',rclpy.time.Time())
			except TransformException as ex:
				self.get_logger().info(f'Could not transform map to camera_optical_joint: {ex}')
				return
			pose = do_transform_pose(pose,t)
			if id not in self.filters:
				self.filters[id] = KalmanFilter()
				
			self.filters[id].updateWithPose(pose) #KF on the global aruco location
			self.ArucoLocations[id] = self.filters[id].get_estimate()
			self.make_transforms('Aruco',id,self.ArucoLocations[id])
			self.make_transforms('Location',id,self.PoseToVisitForGivenAruco(self.ArucoLocations[id], id))
			self.aruco_locations_publisher()
   
		# if all(pose is not None for pose in self.ArucoLocations):
		# 	self.get_logger().info('All arucos have been visited')
		# 	self.aruco_locations_publisher()
		# 	self.destroy_node()
		# 	rclpy.shutdown()

	def aruco_locations_publisher(self):
		#Do we need to publish other informations?
		#See aruco_node to see how it's done (Using ArrayPose())
		locationArucos = PoseArray()
		locationArucos.header.frame_id = "Poses where the arucos are located"
		robotToVisit = PoseArray()
		robotToVisit.header.frame_id = "Poses where the robot should go to visit the aruco"
		for pose in self.ArucoLocations:
			if pose is not None:
				locationArucos.poses.append(pose)
				#robotToVisit.poses.append(PoseToVisitForGivenAruco(pose))
		self.publisher1.publish(locationArucos)
		self.publisher2.publish(robotToVisit)
	
	def make_transforms(self,Name, ID, ArucoPose):
		t = TransformStamped()
		t.header.stamp = self.get_clock().now().to_msg()
		if Name == 'Aruco':
			t.header.frame_id = 'odom'
			t.child_frame_id = Name + str(ID)
		else:
			t.header.frame_id = 'Aruco' + str(ID)
			t.child_frame_id = Name + str(ID)


		t.transform.translation.x = float(ArucoPose.position.x)
		t.transform.translation.y = float(ArucoPose.position.y)
		t.transform.translation.z = float(ArucoPose.position.z)

		t.transform.rotation.x = float(ArucoPose.orientation.x)
		t.transform.rotation.y = float(ArucoPose.orientation.y)
		t.transform.rotation.z = float(ArucoPose.orientation.z)
		t.transform.rotation.w = float(ArucoPose.orientation.w)
		self.tf_static_broadcaster.sendTransform(t)

# def PoseToVisitForGivenAruco(ArucoPose): #To be improved
# 		#What is the orientation in the ArucoPose ?
# 		#Values have been found using simulations on Gazebo
# 		t1 = TransformStamped()
  
# 		t1.transform.translation.x = 0.5
# 		t1.transform.translation.y = 0.0
# 		t1.transform.translation.z = 0.0

# 		# We want to get a vector that face the wall
# 		# We have rotation of the axis of 180 degrees around z axis
# 		# And rotation around y of -90 degrees
# 		t1.transform.rotation.x = 0.0
# 		t1.transform.rotation.y = 0.0
# 		t1.transform.rotation.z = 0.0
# 		t1.transform.rotation.w = 0.0 
  
# 		# t2 = TransformStamped()

# 		# t2.transform.translation.x = 0.0
# 		# t2.transform.translation.y = 0.0
# 		# t1.transform.translation.z = 0.0

# 		# # We want to get a vector that face the wall
# 		# # And rotation around y of -90 degrees ? How come? TBC
# 		# t1.transform.rotation.x = 0.0
# 		# t1.transform.rotation.y = -0.707
# 		# t1.transform.rotation.z = 0.0
# 		# t1.transform.rotation.w = 0.707 
		
# 		#return do_transform_pose(do_transform_pose(ArucoPose, t1),t2)
# 		return do_transform_pose(ArucoPose,t1)

	# def PoseToVisitForGivenAruco(self,ArucoPose, ID):
	# 	# Create the transform to move forward in Aruco's frame
	# 	try:
	# 		t = self.tf_buffer.lookup_transform('camera_optical_joint','map',rclpy.time.Time())
	# 	except TransformException as ex:
	# 		self.get_logger().info(f'Could not transform camera_optical_joint to map: {ex}')
	# 		return
	# 	pose = do_transform_pose(ArucoPose,t)
	# 	pose.position.y += 0.5
		
	# 	try:
	# 		t2 = self.tf_buffer.lookup_transform('map','camera_optical_joint',rclpy.time.Time())
	# 	except TransformException as ex:
	# 		self.get_logger().info(f'Could not transform map to camera_optical_joint: {ex}')
	# 		return

	# 	# Apply this transform relative to the Aruco pose
	# 	return do_transform_pose(pose, t2)
	
	def PoseToVisitForGivenAruco(self,ArucoPose, ID):
		# Create the transform to move forward in Aruco's frame
		ArucoPose.position.x += 0.5

		# Apply this transform relative to the Aruco pose
		return ArucoPose

def main(args=None):
	rclpy.init(args=args)
	aruco_subscriber = ArucoSubscriber()
	rclpy.spin(aruco_subscriber)
	aruco_subscriber.destroy_node() #Check that it's needed or not if node is destroyed in aruco_callback
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()