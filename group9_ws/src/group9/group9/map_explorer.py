import rclpy
import numpy as np
import matplotlib.pyplot as plt
import math
from collections import deque
import time

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatus
from std_msgs.msg import Float32MultiArray

class MapExplorer(Node):

	def __init__(self):
		super().__init__('map_explorer')

		# Subscription to Occupancy map
		self.map_processing = self.create_subscription(
			OccupancyGrid, 
			'map', 
			self.map_processing_callback,
			10)
		self.map_processing

		# Subscription to Robot pose
		self.robot_pose = self.create_subscription(PoseStamped, '/robot_pose', self.robot_pose_callback, 10)

		# Discovering the map every 10sec
		self.map_discovery = self.create_timer(10, self.map_discovery_callback)
		
		self.grid_matrix = np.zeros((2000, 2000), dtype = float) #Processed map
		self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose') #Nav2 Action Client
		self.sent_goal = False #True when robot is processing the goal
		self.explored_map = False #true when explo phase is finished
		self.visited_poses = [] 
		

		# Robot position
		self.x_robot = 0 
		self.y_robot = 0

		# Grid info
		self.p = np.zeros(3)
		self.q = np.zeros(4)
		self.occ_res = 0 
		self.width = 0
		self.height = 0

		# Publishing 360-rotation
		self.publisher_360 = self.create_publisher(
			Twist, 
			'/cmd_vel', #command velocity
			10
		)
		self.publisher_360

		# Publisher for the array of pose visited + 360
		self.publisher_pose = self.create_publisher(
			Float32MultiArray, 
			'/pos_array',
			10
		)
		self.publisher_pose

	# Process the map
	def map_processing_callback(self, grid):
		self.grid_matrix = self.load_grid(grid)

	# Discover map
	def map_discovery_callback(self):
		if(self.sent_goal == False and self.explored_map == False):
			self.sent_goal = True
			self.get_logger().info('Sending goal to the robot')

			# Finding goal 
			go_to_goal , x_goal, y_goal = self.find_next_goal(self.grid_matrix)

			if go_to_goal :
				self.get_logger().info(f" x goal : {x_goal}, y goal : {y_goal}")
				self.rotate_360() 
				self.visited_poses.append([self.x_robot, self.y_robot])
				self.send_goal(x_goal, y_goal)
		return
	
	# Update robot position
	def robot_pose_callback(self, msg):
		self.x_robot = msg.pose.position.x
		self.y_robot = msg.pose.position.y
		return 
			


	##### Function to process the map #####
	def angle_to_quaternion(self, qx, qy, qz, qw):
		return math.atan2(2*(qw * qz + qx * qy), 1-2*(qy**2 + qz**2))

	def index_to_real_world(self, i, j):
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
		theta = self.angle_to_quaternion(self.q[0], self.q[1], self.q[2], self.q[3])
		
		x_loc = (j + 0.5) * self.occ_res
		y_loc = (i + 0.5) * self.occ_res
		
		dx = x_loc * np.cos(theta) - y_loc * np.sin(theta)
		dy = x_loc * np.sin(theta) +  y_loc * np.cos(theta)
		
		x = self.p[0] + dx
		y = self.p[1] + dy
		
		return x , y

	def world_to_index(self, x, y):
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
		theta = self.angle_to_quaternion(self.q[0], self.q[1], self.q[2], self.q[3])

		dx = x - self.p[0]
		dy = y - self.p[1]

		x_loc = dx * np.cos(theta) + dy * np.sin(theta)
		y_loc = dy * np.cos(theta) - dx * np.sin(theta)

		j = int(x_loc / self.occ_res - 0.5)
		i = int(y_loc / self.occ_res - 0.5)

		return i , j
	def load_grid(self, grid):
		self.width = grid.info.width
		self.height = grid.info.height
		origin = grid.info.origin
		self.p = np.array([origin.position.x, origin.position.y, origin.position.z])
		self.q = np.array([origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w])
		self.occ_res = grid.info.resolution
		data = grid.data
		data = np.reshape(data, (self.height, self.width))
		wall_mask  = (data > 60)
		no_mask = (data < 30) & (data > 0)
		unknown = (data >= 30) & (data <= 60)
		data[wall_mask] = 1.0
		data[no_mask] = 0.0
		data[unknown] = -1.0
		completed_mask = np.zeros_like(data, dtype = bool)
		for i in range(self.height):
			for j in range(self.width):
				if i<self.height-1 and j<self.width-1:
					nb_one = 0
					if data[i,j] == 1:
						if data[i-1,j] == 0 and data[i+1,j] == 0 and data[i,j-1] == 0 and data[i,j+1] == 0:
							data[i,j] = 0
					else: 
						if data[i-1,j-1] == 1:
							nb_one += 1 
						if data[i-1,j] == 1:
							nb_one += 1 
						if data[i-1,j+1] == 1:
							nb_one += 1  
						if data[i,j-1] == 1:
							nb_one += 1
						if data[i,j+1] == 1:
							nb_one += 1  
						if data[i+1,j-1] == 1:
							nb_one += 1 
						if data[i+1,j] == 1:
							nb_one += 1  
						if data[i+1,j+1] == 1:
							nb_one += 1 
						if nb_one >= 2:
							completed_mask[i,j] = True                  

		data[completed_mask] = 1
		return data
	

	##### Functions to discover the map ######

	# Sending goal
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

	# Accept or rejet node
	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected')
			return

		self.get_logger().info('Goal accepted')
		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.get_result_callback)

	# Goal reached or failed
	def get_result_callback(self, future):
		result = future.result().result
		status = future.result().status
		if status == GoalStatus.STATUS_SUCCEEDED :
			self.get_logger().info('Goal succeeded !')
			
		else :
			self.get_logger().info('Goal failed.')
		
		self.sent_goal = False

	# Finds next goal position
	def find_next_goal(self, grid_mtx):
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
			flat_array = self.visited_poses.flatten()
			msg = Float32MultiArray()
			msg.data = flat_array.tolist()
			self.publisher_pose.publish(msg)
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

		frontier_middles_coord = []
		for i, j in frontier_middles :
			x, y = self.index_to_real_world(i,j)
			frontier_middles_coord.append((x,y))
		

		# Finding closest goal
		min_dist = np.inf
		for x, y in frontier_middles_coord :
			if np.sqrt((x-self.x_robot)**2 + (y-self.y_robot)**2) < min_dist:
				min_dist = np.sqrt((x-self.x_robot)**2 + (y-self.y_robot)**2)
				x_goal = x
				y_goal = y

		return True, x_goal, y_goal
	
	#360 rotation to see Arucos
	def rotate_360(self):
		angular_speed = 0.3 #rad/s
		twist = Twist()
		twist.linear.x = 0.0
		twist.angular.z = angular_speed

		angle = 2*4 #360°
		duration = angle/angular_speed
		start_time = self.get_clock().now().seconds_nanoseconds()[0]
		self.get_logger().info("Starting rotation")

		while rclpy.ok():
			current_time = self.get_clock().now().seconds_nanoseconds()[0]
			if current_time-start_time >= duration:
				break
			self.publisher_360.publish(twist)
			time.sleep(0.1)
		
		twist.angular.z = 0.0
		self.publisher_360.publish(twist)
		self.get_logger().info("Finished spinning, now going to goal !")

	### Function to do the raycast
	def one_ray_cast(self, r_pos_map, angle):

		i_max = self.height
		j_max = self.width

		rd = [np.cos(angle), np.sin(angle)]
		
		step = np.sign(rd)

		
		theta = self.angle_to_quaternion(self.q[0], self.q[1], self.q[2], self.q[3])
		dx = r_pos_map[0] - self.p[0]
		dy = r_pos_map[1] - self.p[1]

		x_loc = dx * np.cos(theta) + dy * np.sin(theta)
		y_loc = dy * np.cos(theta) - dx * np.sin(theta)

		j_float = x_loc / self.occ_res - 0.5
		i_float = y_loc / self.occ_res - 0.5
		j = math.floor(j_float)
		i = math.floor(i_float)

		x_test, y_test = self.index_to_real_world(i, j)

		self.get_logger().info(f'i = {i}')
		self.get_logger().info(f'j = {j}')
		self.get_logger().info(f'x_robot = {r_pos_map[0]}')
		self.get_logger().info(f'y_robot = {r_pos_map[1]}')

		self.get_logger().info(f'x_test = {x_test}')
		self.get_logger().info(f'y_test = {y_test}')
		frac = [j_float - j, i_float - i] 

		unit_step = [np.sqrt(1 + (rd[1]/rd[0])**2)if rd[0] != 0 else float('inf'),np.sqrt(1 + (rd[0]/rd[1])**2)if rd[1] != 0 else float('inf') ]
		#unit_step = [ abs(1 / rd[0]) if rd[0] != 0 else float('inf'),abs(1 / rd[1]) if rd[1] != 0 else float('inf')]


		ray_length = np.zeros(2)
		if (rd[0] < 0):
			ray_length[0] = frac[0] * unit_step[0]
		else:
			ray_length[0] = (1 - frac[0]) * unit_step[0]

		if (rd[1] < 0):
			ray_length[1] = frac[1] * unit_step[1]
		else:
			ray_length[1] = (1 - frac[1]) * unit_step[1]

		true_xlen = ray_length[0] * self.occ_res
		true_ylen = ray_length[1] * self.occ_res
		true_length = np.sqrt(true_xlen**2 + true_ylen ** 2)

		i_prev = i
		j_prev = j

		self.get_logger().info(f'true_length = {true_length}')

		while true_length < 2:
			if i < 0 or j < 0 or i >= i_max - 1 or j >= j_max- 1:
				return False, None, None
			
			
			if self.grid_matrix[i,j] == 1.0:
				return True, i_prev, j_prev
			
			else: 
				i_prev = i
				j_prev = j
				if (ray_length[0] < ray_length[1]):
					j = int( j + step[0])
					ray_length[0] += unit_step[0]
				
				else:       
					i = int(i + step[1])
					ray_length[1] += unit_step[1]
			true_xlen = ray_length[0] * self.occ_res
			true_ylen = ray_length[1] * self.occ_res
			true_length = np.sqrt(true_xlen**2 + true_ylen ** 2)

		return False, 0, 0

	def raycast_360(self, n_rays=300):
		r_pos_map = [self.x_robot, self.y_robot]
		for n in range(n_rays):
			if n==0:
				continue
			self.get_logger().info(f'n = {n}')
			angle = n * 2 * np.pi / n_rays
			hit, i, j = self.one_ray_cast(r_pos_map, angle)
			if hit:
				self.get_logger().info(f'i = {i}')
				self.get_logger().info(f'j = {j}')

				self.grid_matrix[i,j] = 2.0 #à voir ce qu'on veut faire avec

def main(args=None):
	rclpy.init(args=args)
	map_explorer = MapExplorer()
	rclpy.spin(map_explorer)
	map_explorer.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
