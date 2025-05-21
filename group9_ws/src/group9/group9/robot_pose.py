import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener


class RobotPosition(Node):
	def __init__(self):
		super().__init__('robot_position')
		self.publisher_ = self.create_publisher(PoseStamped, '/robot_pose', 10)
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.timer = self.create_timer(0.5, self.publish_pose)
	
	def publish_pose(self):
		try:
			transform = self.tf_buffer.lookup_transform(
				'base_footprint',
				'map',
				rclpy.time.Time())
			pose = PoseStamped()
			pose.header = transform.header
			pose.pose.position.x = transform.transform.translation.x
			pose.pose.position.y = transform.transform.translation.y
			pose.pose.position.z = transform.transform.translation.z
			pose.pose.orientation = transform.transform.rotation
			self.publisher_.publish(pose)
			self.get_logger().info('Found robot position')
		except TransformException as ex:
			self.get_logger().info(f'Could not transform "base_footprint" to "map": {ex}')
			
def main(args=None):
	rclpy.init(args=args)
	robot_position = RobotPosition()
	rclpy.spin(robot_position)
	robot_position.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
