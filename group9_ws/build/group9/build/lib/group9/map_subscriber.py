import rclpy
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.ndimage import binary_dilation

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class MapSubscriber(Node):

	def __init__(self):
		super().__init__('map_subscriber')
		self.subscription = self.create_subscription(
			OccupancyGrid, 
			'map', 
			self.listener_callback,
			10)
		self.subscription
		self.grid_matrix = np.zeros((2000, 2000))
		self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
		self.reached_goal = True
		
	def listener_callback(self, grid):
		self.get_logger().info('Occupancy grid management')
		
		self.grid_matrix = self.load_grid(grid)

		np.savetxt("matrix.txt", self.grid_matrix)
		
		plt.imshow(self.grid_matrix, cmap="viridis")
		plt.colorbar()
		plt.savefig("src/group9/heatmapNODILATATION.pdf", dpi=300, bbox_inches='tight')
		plt.close()

		if(self.reached_goal == False):
			self.reached_goal = True
			self.get_logger().info('Sending goal to the robot')
			origin = grid.info.origin
			p = np.array([origin.position.x, origin.position.y, origin.position.z])
			q = np.array([origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w])
			x_goal, y_goal = self.index_to_real_world(20,50, grid.info.resolution, p, q)
			self.get_logger().info(f" x goal : {x_goal}, y goal : {y_goal}")
			self.send_goal(x_goal, y_goal)
			

		
	def angle_to_quaternion(self, qx, qy, qz, qw):
		return math.atan2(2*(qw * qz + qx * qy), 1-2*(qy**2 + qz**2))

	def index_to_real_world(self, i, j, res, p, q):
		""" Fonction that transform the index of the Occupancy Grid to a real world position
			where the real world origin is determined when the robot is launched
			
			argument:
				- i: index of the column in the Occupancy Grid
				- j: index of the line in the Occupancy Grid
				- res: resolution of the Occupancy Grid
				- p: vector of 3 dimensions = coordinate of the origin of the Occupancy Grid
				- q: vector of 4 dimensions = quaternion of the Occupancy Grid with respect to the real world
				
			return: 
				x, y: the coordinate of the cellule(j,i) in the real world
		"""
		theta = self.angle_to_quaternion(q[0], q[1], q[2], q[3])
		
		x_loc = (i + 0.5) * res
		y_loc = (j + 0.5) * res
		
		dx = x_loc * np.cos(theta) - y_loc * np.sin(theta)
		dy = x_loc * np.sin(theta) +  y_loc * np.cos(theta)
		
		x = p[0] + dx
		y = p[1] + dy
		
		return x , y

	def load_grid(self, grid):
		width = grid.info.width
		height = grid.info.height
		data = grid.data
		data = np.reshape(data, (height, width))
		origin = grid.info.origin

		wall_mask  = (data > 60)
		no_mask = (data < 30) & (data > 0)
		unknown = (data >= 30) & (data <= 60)
		data[wall_mask] = 1.0
		data[no_mask] = 0.0
		data[unknown] = -1
		unknown_mask = (data == -1)
		data[unknown_mask] = 0.0
		for i in range(height):
			for j in range(width):
				if i<height-1 and j<width-1:
					if data[i,j] == 1:
							if data[i-1,j] == 0 and data[i+1,j] == 0 and data[i,j-1] == 0 and data[i,j+1] == 0:
								data[i,j] = 0
					else: 
						if data[i-1,j] == 1 and data[i+1,j] == 1 and data[i,j-1] == 1 and data[i,j+1] == 1:
							data[i,j] = 1                  
		unknown_mask = unknown_mask & (data == 0)
		final_grid = data
		#final_grid= binary_dilation(data, iterations=1)
		final_grid = final_grid.astype(int)
		final_grid[unknown_mask] = -1
		
		return final_grid
	
	def send_goal(self, x, y):
		self.get_logger().info('Waiting for action server..')
		self._action_client.wait_for_server()
		goal_msg = NavigateToPose.Goal()

		goal_msg.pose = PoseStamped()
		goal_msg.pose.header.frame_id = "map" #-> move wrt to this frame
		goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
		goal_msg.pose.pose.position.x = x #In meters !
		goal_msg.pose.pose.position.y = y
		goal_msg.pose.pose.orientation.w = 1.0
		goal_msg.behavior_tree =""

		self.get_logger().info('Sending goal request..')

		self._send_goal_future = self._action_client.send_goal_async(goal_msg)

		self._send_goal_future.add_done_callback(self.goal_response_callback)

	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected')
			return

		self.get_logger().info('Goal accepted')
		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.get_result_callback)

	def get_result_callback(self, future):
		result = future.result().result
		status = future.result().status
		if status == GoalStatus.STATUS_SUCCEEDED :
			self.get_logger().info('Goal succeeded !')
			self.reached_goal = True
			
		else :
			self.get_logger().info('Goal failed.')

def main(args=None):
	rclpy.init(args=args)
	map_subscriber = MapSubscriber()
	rclpy.spin(map_subscriber)
	map_subscriber.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
