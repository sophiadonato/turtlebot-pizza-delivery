import rclpy
import numpy as np
import matplotlib.pyplot as plt
import math
from collections import deque
from scipy.ndimage import binary_dilation

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseArray
from action_msgs.msg import GoalStatus
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener


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
		self.subscriptionAruco = self.create_subscription(PoseArray, 'arucos_location', self.arruco_callback, 10)
		self.subscriptionArucoRobot = self.create_subscription(PoseArray, 'robot_location', self.arrucoRobot_callback, 10)
		self.sent_goal = False
		self.explored_map = False
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.x_robot = 0
		self.y_robot = 0
		self.resolution = 0
		self.p = [0,0]
		self.q = [0,0]
		self.listArrucoVisit = []
		self.listArrucoRobot = []
		
	def listener_callback(self, grid):
		# self.get_logger().info('Occupancy grid management')
		
		self.grid_matrix = self.load_grid(grid)
		self.resolution = grid.info.resolution
		self.p = grid.info.origin.position
		self.q = grid.info.origin.orientation
		for elem in self.listArrucoRobot:
			self.grid_matrix[elem[0]][elem[1]] = 1.5
		for elem in self.listArrucoVisit:
			self.grid_matrix[elem[0]][elem[1]] = 2
		np.savetxt("matrix.txt", self.grid_matrix)
		
		plt.imshow(self.grid_matrix, cmap="viridis")
		plt.colorbar()
		plt.savefig("src/group9/heatmapTestAruco.pdf", dpi=300, bbox_inches='tight')
		plt.close()
			
	def arruco_callback(self, msg):
		listOfPoses = msg.poses
		value = 0
		for pose in listOfPoses:
			i,j = self.world_to_index(pose.position.x, pose.position.y,self.resolution,self.p, self.q)
			self.get_logger().info(f" ARUCO {value} :i : {i}, j: {j}")
			value += 1
			self.listArrucoVisit.append((i,j))


	def arrucoRobot_callback(self, msg):
		listOfPoses = msg.poses
		value = 0
		for pose in listOfPoses:
			i,j = self.world_to_index(pose.position.x, pose.position.y,self.resolution,self.p, self.q)
			self.get_logger().info(f" ROBOT {value}: i : {i}, j: {j}")
			value += 1
			self.listArrucoRobot.append((i,j))
		
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
		theta = self.angle_to_quaternion(q.x, q.y, q.z, q.w)
		
		x_loc = (i + 0.5) * res
		y_loc = (j + 0.5) * res
		
		dx = x_loc * np.cos(theta) - y_loc * np.sin(theta)
		dy = x_loc * np.sin(theta) +  y_loc * np.cos(theta)
		
		x = p[0] + dx
		y = p[1] + dy
		
		return x , y
	
	def world_to_index(self,x, y, res, p, q):
		""" Fonction that transform the index of the Occupancy Grid to a real world position
			where the real world origin is determined when the robot is launched
			
			argument:
				- x: x-coordinate in the real world
				- y: y-coordinate in the real world
				- res: resolution of the Occupancy Grid
				- p: vector of 3 dimensions = coordinate of the origin of the Occupancy Grid
				- q: vector of 4 dimensions = quaternion of the Occupancy Grid with respect to the real world
				
			return: 
				i, j: index of the point (x,y) in the Occupancy grid

			/!\: in Occupancy Grid, the cell [0,0] is the low left corner.
		"""
		theta = self.angle_to_quaternion(q.x, q.y, q.z, q.w)

		dx = x - p.x
		dy = y - p.y

		x_loc = dx * np.cos(theta) + dy * np.sin(theta)
		y_loc = dy * np.cos(theta) - dx * np.sin(theta)

		j = int(x_loc / res - 0.5)
		i = int(y_loc / res - 0.5)

		return i , j

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
	

	def get_result_callback(self, future):
		result = future.result().result
		status = future.result().status
		if status == GoalStatus.STATUS_SUCCEEDED :
			self.get_logger().info('Goal succeeded !')
			
		else :
			self.get_logger().info('Goal failed.')

		self.sent_goal = True

	def find_next_goal(self, grid_mtx, grid):
		while rclpy.ok() :
			try:
				transform = self.tf_buffer.lookup_transform(
					'base_footprint',
					'map',
					rclpy.time.Time())
				break
			except TransformException as ex:
				self.get_logger().info(f'Could not transform "base_footprint" to "map": {ex}')
				rclpy.spin_once(self, timeout_sec=0.1)
		
		self.x_robot = transform.transform.translation.x
		self.y_robot = transform.transform.translation.y
		self.get_logger().info(f'x position of the robot: {self.x_robot}')
		self.get_logger().info(f'y position of the robot: {self.y_robot}')


		frontier_cells = []
		neighboors = [(-1, 0), (1,0), (0,-1), (0,1), (-1,-1), (-1, 1), (1, 1), (1, -1)]

		# Finding all frontier cells
		for i in range(grid_mtx.shape[0]):
			for j in range(grid_mtx.shape[1]):
				if grid_mtx[i][j] == 0:
					is_frontier = False
					next_to_wall = False
					for di,dj in neighboors:
						neighboor_i, neighboor_j = i+di, j+dj
						if 0<= neighboor_i < grid_mtx.shape[0] and 0<= neighboor_j < grid_mtx.shape[1]:
							if grid_mtx[neighboor_i][neighboor_j] == -1 :
								is_frontier = True
							if grid_mtx[neighboor_i][neighboor_j] == 1:
								next_to_wall = True
					if is_frontier and not next_to_wall:
						frontier_cells.append((i,j))

		
		# Putting cells into different frontiers
		frontiers = []
		visited = set()

		for cell in frontier_cells:
			if cell not in visited:
				frontier = []
				queue = deque([cell])
				visited.add(cell)

				while queue:
					current_cell = queue.pop()
					frontier.append(current_cell)
					for di,dj in neighboors:
						neighboor_i, neighboor_j = current_cell[0]+di, current_cell[1]+dj
						if 0<= neighboor_i < grid_mtx.shape[0] and 0<= neighboor_j < grid_mtx.shape[1]:
							if (neighboor_i, neighboor_j) in frontier_cells and (neighboor_i, neighboor_j) not in visited:
								visited.add((neighboor_i, neighboor_j))
								queue.append((neighboor_i, neighboor_j))
				frontiers.append(frontier)

		# Removing frontier whose length is too small 
		# Frontier could be too small because of 2 reasons : "bad" prepocessing, or too close of a wall
		frontiers_copy = frontiers.copy()
		for frontier in frontiers_copy:
			if len(frontier) < 10:
				frontiers.remove(frontier)

		# If not frontiers map explo is finished
		if len(frontiers) <= 0 :
			self.explored_map = True
			self.get_logger().info(f'Finished map')
			return False, 0, 0

		# Finding middle of frontiers
		frontier_middles = []
		for frontier in frontiers : 
			frontier = np.array(frontier)
			centroid = np.mean(frontier, axis=0)
			min_dist = np.inf
			for i, j in frontier:
				if np.sqrt((i-centroid[0])**2 + (j-centroid[1])**2) < min_dist:
					min_dist = np.sqrt((i-centroid[0])**2 + (j-centroid[1])**2)
					middle = (i,j)
			frontier_middles.append(middle)

		

		# Getting true coordinates of each middle
		origin = grid.info.origin
		self.p = np.array([origin.position.x, origin.position.y, origin.position.z])
		self.q = np.array([origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w])

		frontier_middles_coord = []
		for i, j in frontier_middles :
			x, y = self.index_to_real_world(j,i, grid.info.resolution, self.p, self.q)
			frontier_middles_coord.append((x,y))
		

		# Finding closest goal
		min_dist = np.inf
		for x, y in frontier_middles_coord :
			if np.sqrt((x-self.x_robot)**2 + (y-self.x_robot)**2) < min_dist:
				min_dist = np.sqrt((x-self.x_robot)**2 + (y-self.y_robot)**2)
				x_goal = x
				y_goal = y

		plt.imshow(self.grid_matrix, cmap="viridis")
		plt.colorbar()
		plt.savefig("src/group9/map.pdf", dpi=300, bbox_inches='tight')
		plt.close()

		return True, x_goal, y_goal

def main(args=None):
	rclpy.init(args=args)
	map_subscriber = MapSubscriber()
	rclpy.spin(map_subscriber)
	map_subscriber.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
