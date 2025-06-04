import numpy as np
import math
import rclpy
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Pose, PoseArray,TransformStamped
from std_msgs.msg import Int32
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
		self.publisherNumber = self.create_publisher(Int32, '/arucos_number', 10)
		self.publisher = self.create_publisher(PoseArray, '/arucos_location', 10)
		self.ArucoLocations = [None]*nbArucos
		self.LocationVisit = [None]*nbArucos
		self.nbVisits = np.zeros(nbArucos)

		#We implement a Kalman filter to get rid of camera noise
		self.filtersAruco = {}
		
		#Broadcaster
		self.tf_static_broadcaster = StaticTransformBroadcaster(self)

	def aruco_callback(self, arucoMarkers):
		for i in range(0, len(arucoMarkers.marker_ids)):
			id = arucoMarkers.marker_ids[i]
			self.get_logger().info(f'Aruco information : ID : {id}')
			if id > 10:
				self.get_logger().info(f'This aruco cannot be found on the map and is discarded')
				break
			
			pose = arucoMarkers.poses[i]
			try:
				t = self.tf_buffer.lookup_transform('map','camera_optical_joint',rclpy.time.Time())
			except TransformException as ex:
				self.get_logger().info(f'Could not transform map to camera_optical_joint: {ex}')
				return
			
			pose = do_transform_pose(pose,t)
			if id not in self.filtersAruco:
				self.filtersAruco[id] = KalmanFilter()
				
			self.filtersAruco[id].updateWithPose(pose)

			self.ArucoLocations[id] = self.filtersAruco[id].get_estimate()
			self.nbVisits[id] += 1
			self.LocationVisit[id] = self.robotLocation(self.ArucoLocations[id])
			self.make_transforms('Aruco',id,self.ArucoLocations[id])
			self.make_transforms('Location',id,self.LocationVisit[id])
			
		msg = Int32()
		msg.data = sum(1 for item in self.LocationVisit if item is not None)
		self.get_logger().info(f'Number of arucos already found {msg.data}')
		self.publisherNumber.publish(msg)
		
		if all(value > 5 for value in self.nbVisits):
			self.get_logger().info('All arucos have been visited')
			self.aruco_locations_publisher()

	def aruco_locations_publisher(self):
		robotToVisit = PoseArray()
		robotToVisit.header.frame_id = "Poses where the robot should go to visit the aruco"
		for pose in self.LocationVisit:
			if pose is not None:
				robotToVisit.poses.append(pose)
		self.publisher.publish(robotToVisit)
	
	def make_transforms(self,Name, ID, ArucoPose):
		t = TransformStamped()
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = 'map'
		t.child_frame_id = Name + str(ID)
		t.transform.translation.x = float(ArucoPose.position.x)
		t.transform.translation.y = float(ArucoPose.position.y)
		t.transform.translation.z = float(ArucoPose.position.z)

		t.transform.rotation.x = float(ArucoPose.orientation.x)
		t.transform.rotation.y = float(ArucoPose.orientation.y)
		t.transform.rotation.z = float(ArucoPose.orientation.z)
		t.transform.rotation.w = float(ArucoPose.orientation.w)
		self.tf_static_broadcaster.sendTransform(t)

	def robotLocation(self,ArucoPose):
		robotPose = Pose()
		yaw = quaternion_to_yaw(ArucoPose.orientation.x,ArucoPose.orientation.y,ArucoPose.orientation.z, ArucoPose.orientation.w) 
		robotPose.position.x = ArucoPose.position.x + 0.5*math.sin(yaw)
		robotPose.position.y = ArucoPose.position.y - 0.5*math.cos(yaw)
		robotPose.position.z = ArucoPose.position.z
		original_q = R.from_quat([ArucoPose.orientation.x,
                          ArucoPose.orientation.y,
                          ArucoPose.orientation.z,
                          ArucoPose.orientation.w])
		rot_z_90 = R.from_euler('z', 90, degrees=True)
		rot_y_90 = R.from_euler('y', 90, degrees=True)
		composite_rotation = rot_z_90*rot_y_90
		rotated_q = original_q*composite_rotation
		robotPose.orientation.x,robotPose.orientation.y, robotPose.orientation.z,robotPose.orientation.w = rotated_q.as_quat()
		return robotPose
	

def quaternion_to_yaw(x,y,z,w):
	siny_cosp = 2 * (w * z + x * y)
	cosy_cosp = 1 - 2 * (y * y + z * z)
	return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
	rclpy.init(args=args)
	aruco_subscriber = ArucoSubscriber()
	rclpy.spin(aruco_subscriber)
	aruco_subscriber.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()