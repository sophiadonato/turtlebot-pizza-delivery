import rclpy
import numpy as np
import math
from collections import deque
import matplotlib.pyplot as plt
import random
import time

from std_msgs.msg import Int32
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist



class Raycaster(Node): 
    def __init__(self):
        super().__init__('last_arucos')

        # Subscription to the map
        self.map_processing_subscription = self.create_subscription(OccupancyGrid, 'map', self.map_processing_callback,10)
        self.map_processing_subscription
		
		# Subscription to the vector of already visited position
        self.poses_to_raycast_subscription = self.create_subscription(Float32MultiArray, '/pos_array', self.poses_to_raycast_callback,10)
        self.poses_to_raycast_subscription
        self.pos_array = []
        self.start = False #True when receiving the positions from mapExplorer node

        # Subscription to the number of aruco
        self.nbr_arucos_subscriber = self.create_subscription(Int32,'/arucos_number', self.load_nbr_aruco_callback, 10)
        self.nbr_arucos_subscriber
        self.nbr_aruco = 0
	
        # Subscription to the robot pose (needed to raycast the new discovered area)
        self.robot_pose = self.create_subscription(PoseStamped, '/robot_pose', self.robot_pose_callback, 10)

        # Publishing 360-rotation
        self.publisher_360 = self.create_publisher(
            Twist, 
            '/cmd_vel', #command velocity
            10
        )
        self.publisher_360

        # Doing raycast every 3 sec
        self.raycast = self.create_timer(3, self.raycast_callback)
        self.raycast_done = False
        self.goals = []
        self.flag_first = True

        # Discovering the map every 3 sec
        self.map_discovery = self.create_timer(3, self.map_discovery_callback)

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.sent_goal = False #True when robot is processing the goal

        # Robot position
        self.x_robot = 0 
        self.y_robot = 0
        self.z_robot = 0.0

        self.raycast_matrix = np.zeros((2000, 2000), dtype = float)
        self.grid_no_raycast = np.zeros((2000, 2000), dtype = float)
        self.p = np.zeros(3)
        self.q = np.zeros(4)
        self.occ_res = 0 
        self.width = 0
        self.height = 0


    #To load the grid one time
    def map_processing_callback(self, grid):
        self.grid_no_raycast = self.load_grid(grid)
        plt.imshow(self.grid_no_raycast, cmap="viridis")
        plt.colorbar()
        plt.savefig("src/group9/first_loaded_map.pdf", dpi=300, bbox_inches='tight')
        plt.close()
    
    # Update robot position
    def robot_pose_callback(self, msg):
        self.x_robot = msg.pose.position.x
        self.y_robot = msg.pose.position.y
        self.z_robot = msg.pose.position.z
        return 
    
    # Load information about aruco (again see with arnaud)
    def load_nbr_aruco_callback(self, msg):
        self.nbr_aruco = msg.data
    
    # Receiving first array from map explorer, putting it in good shape 
    def poses_to_raycast_callback(self, msg) :
        if self.nbr_aruco < 10:
            flat_data = np.array(msg.data)
            self.pos_array = flat_data.reshape(-1, 2)
            self.start = True
        else :
            self.get_logger().info('All arucos found !')
        self.destroy_subscription(self.poses_to_raycast_subscription)
        return
    
    def raycast_callback(self):

        if not self.start :
            return 
        if self.nbr_aruco >=10:
            self.get_logger().info('All arucos found ! ')
            return 
        
        self.raycast_matrix = np.copy(self.grid_no_raycast)
        for i in range(self.pos_array.shape[0]):
            self.raycast_360(self.pos_array[i])
        plt.imshow(self.raycast_matrix, cmap="viridis")
        plt.colorbar()
        plt.savefig("src/group9/raycasting_map.pdf", dpi=300, bbox_inches='tight')
        plt.close()

        self.goals = self.find_goals(self.raycast_matrix) #PB IF NO GOAL AND NOT ALL ARUCOS
        self.nb_goals = len(self.goals)
        self.goal_idx = 0

        self.raycast_done = True
        self.sart = False

    def map_discovery_callback(self):
        if not self.raycast_done:
            return
        
        if self.nbr_aruco >=10:
            self.get_logger().info('All arucos found ! ')
            return
        
        if self.sent_goal == False: #and self.goal_idx < self.nb_goals:
            self.sent_goal = True
            if self.flag_first:
                self.flag_first = False
            else : 
                self.rotate_360()
            self.get_logger().info('Sending goal to the robot')
            if self.nb_goals > 0:
                min_dist = np.inf
                for x, y in self.goals :
                    if np.sqrt((x-self.x_robot)**2 + (y-self.y_robot)**2) < min_dist:
                        min_dist = np.sqrt((x-self.x_robot)**2 + (y-self.y_robot)**2)
                        x_goal = x
                        y_goal = y
                self.send_goal([x_goal, y_goal])
                self.raycast_done = False
                self.start = True
            else: 
                self.get_logger().info('Raycast complete but not all arucos found')
                return

    ##### Functions to process the grid 

    #One raycast
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
        
        #perimiter of 2 meters, can be modified
        while true_length < 1.5:
            if i < 0 or j < 0 or i >= i_max - 1 or j >= j_max- 1:
                return False, None, None
            
            
            if self.raycast_matrix[i,j] == 1.0:
                return 
            
            else: 
                self.raycast_matrix[i,j] = 2.0
                
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
    
    #360 raycast mark all cells in the raycast with a "2"
    def raycast_360(self, r_pos_map, n_rays=320):
        for n in range(n_rays):
            if n==0:
                continue
            angle = n * 2 * np.pi / n_rays
            self.one_ray_cast(r_pos_map, angle)
        return
    
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

    def angle_to_quaternion(self, qx, qy, qz, qw):
        return math.atan2(2*(qw * qz + qx * qy), 1-2*(qy**2 + qz**2))

    # Transform index of grid to real world position wrt to map
    def index_to_real_world(self, i, j):
        theta = self.angle_to_quaternion(self.q[0], self.q[1], self.q[2], self.q[3])
        
        x_loc = (j + 0.5) * self.occ_res
        y_loc = (i + 0.5) * self.occ_res
        
        dx = x_loc * np.cos(theta) - y_loc * np.sin(theta)
        dy = x_loc * np.sin(theta) +  y_loc * np.cos(theta)
        
        x = self.p[0] + dx
        y = self.p[1] + dy
        
        return x , y

    # Finds goals
    def find_goals(self, raycast_matrix):
        frontier_cells = []
        neighboors = [(-1, 0), (1,0), (0,-1), (0,1), (-1,-1), (-1, 1), (1, 1), (1, -1)]

        # Finding all frontier cells
        for i in range(raycast_matrix.shape[0]):
            for j in range(raycast_matrix.shape[1]):
                if raycast_matrix[i][j] == 0:
                    is_frontier = False
                    next_to_wall = False
                    for di,dj in neighboors:
                        neighboor_i, neighboor_j = i+di, j+dj
                        if 0<= neighboor_i < raycast_matrix.shape[0] and 0<= neighboor_j < raycast_matrix.shape[1]:
                            #modified to detect frontier between raycasted and free cells 
                            if raycast_matrix[neighboor_i][neighboor_j] == 2 :
                                is_frontier = True
                            if raycast_matrix[neighboor_i][neighboor_j] == 1:
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
                        if 0<= neighboor_i < raycast_matrix.shape[0] and 0<= neighboor_j < raycast_matrix.shape[1]:
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


        plt.figure()
        plt.imshow(raycast_matrix, cmap='viridis')
        plt.colorbar()
        for frontier in frontiers:
            color = [random.random() for _ in range(3)]  
            xs = [cell[1] for cell in frontier]  
            ys = [cell[0] for cell in frontier]  
            plt.scatter(xs, ys, c=[color], s=10, label=f'Frontier {frontiers.index(frontier)}')

        for pt in frontier_middles :
            xs = pt[1]
            xy = pt[0]
            plt.scatter(xs, xy, c="black")
        plt.savefig("src/group9/raycasting_frontiers.pdf", dpi=300, bbox_inches='tight')
        plt.close()
        return frontier_middles_coord
    

    # Sending goal
    def send_goal(self, goal):
        self.get_logger().info('Waiting for action server..')
        self._action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map" 
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal[0]#In meters !
        goal_msg.pose.pose.position.y = goal[1]
        goal_msg.pose.pose.position.z = self.z_robot
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

            self.goal_idx +=1
            
        else :
            self.get_logger().info('Goal failed.')
        self.pos_array = np.append(self.pos_array, [[self.x_robot, self.y_robot]], axis=0)
        self.sent_goal = False

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



def main(args=None):
	rclpy.init(args=args)
	raycaster = Raycaster()
	rclpy.spin(raycaster)
	raycaster.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()