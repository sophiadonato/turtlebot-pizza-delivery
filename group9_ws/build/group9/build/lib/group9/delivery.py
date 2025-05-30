# Solution with a separate node for the computation of the path (avoid blocking problems)
import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray,PoseStamped
from std_msgs.msg import Int32MultiArray
from python_tsp.exact import solve_tsp_dynamic_programming #pip install python-tsp
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient

class PathOfDelivery(Node):

    def __init__(self):
        super().__init__('path_of_delivery') 

        # Subscription to Robot pose
        self.subscription_robot_pose = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.robot_pose_callback,
            10)
        self.subscription_robot_pose
        self.robot_pose = None


        # Subscription to Arucos locations
        self.subscription_arucos_location = self.create_subscription(
            PoseArray,
            '/arucos_location',
            self.arucos_location_callback,
            10)
        self.subscription_arucos_location

        # Subscription to Arucos ID to visit (launched by profs)
        self.subcription_IDs = self.create_subscription(
            Int32MultiArray,
            '/delivery_locations',
            self.delivery_location_callback,
            10)
        self.subcription_IDs
        

        # Publisher of ordered Arucos location
        self.publisher = self.create_publisher(PoseArray, '/ordered_aruco_poses', 10)
        self.arucos_pose = []
        self.arucos_to_visit = []

        # Action client node to compute path to pose
        self.compute_path_node = ComputePathBetweenLocations()

        # Action client to deliver
        self.delivery_node = Delivery()

    # Sets arucos pose
    def arucos_location_callback(self, msg):
        self.arucos_pose = msg.poses
        self.destroy_subscription(self.subscription_arucos_location)

    # Gets current robot location (isn't supposed to move in that part)
    def robot_pose_callback(self, msg):
        self.robot_pose = msg.pose
        self.destroy_subscription(self.subscription_robot_pose)
    
    # Gets arucos pose to visit
    def delivery_location_callback(self, msg):
        if all(pose is not None for pose in self.arucos_pose) and self.robot_pose != None:
            ids = msg.data
            self.nb_aruco_to_visit = len(ids)
            for i in ids:
                self.arucos_to_visit = np.append(self.arucos_to_visit,np.array([self.arucos_pose[i]]))
            self.destroy_subscription(self.subcription_IDs)
            self.computation()
    
    # Solves TSP problem, orders arucos position with respect to solution of TSP, published ordered positions
    def computation(self):
        distance_matrix = np.zeros((self.nb_aruco_to_visit + 1,self.nb_aruco_to_visit + 1))
        robot_location = np.array(self.robot_pose)
        poses = np.append(robot_location , self.arucos_to_visit)

        for i in range(0,self.nb_aruco_to_visit +1): 
            for j in range(i + 1, self.nb_aruco_to_visit + 1):
                distance_matrix[i][j] = self.compute_estimated_distance(poses[i], poses[j])
                distance_matrix[j][i] = distance_matrix[i][j]

        distance_matrix[:,0] = 0 
        self.get_logger().info(f'Computing delivery order using TSP')
        permutation,_ = solve_tsp_dynamic_programming(distance_matrix)
        self.get_logger().info(f'Permutation : {permutation}')
        permutation = permutation[1:] #The first element is always the current location of the Robot
        sequence = PoseArray()
        sequence.header.frame_id = 'map'
        for nb in permutation:
            sequence.poses.append(self.arucos_to_visit[nb -1])

        self.publisher.publish(sequence)
        rclpy.spin(self.delivery_node)

    
    # Computes the estimated distance between 2 poses, 
    # Using nav2 Action client that computes a path of poses between start and end and computing an estimated distance between each pose of the path
    def compute_estimated_distance(self, start, end):
        self.compute_path_node.start_pose = start
        self.compute_path_node.end_pose = end
        self.compute_path_node.path_computed = False

        self.get_logger().info(f'Compute path using Action Client node')
        self.compute_path_node.get_path()
        while self.compute_path_node.path_computed == False:
            rclpy.spin_once(self.compute_path_node, timeout_sec = 3)


        distance = 0.0
        for i in range(1, len(self.compute_path_node.path)):
            x0 = self.compute_path_node.path[i - 1].pose.position.x
            y0 = self.compute_path_node.path[i - 1].pose.position.y
            x1 = self.compute_path_node.path[i].pose.position.x
            y1 = self.compute_path_node.path[i].pose.position.y
            distance += math.hypot(x1 - x0, y1 - y0)
        self.get_logger().info(f'Estimated distance : {distance}')
        return distance
    

class ComputePathBetweenLocations(Node):
    def __init__(self):
        super().__init__('compute_path_btw_locations')
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.start_pose = None
        self.end_pose = None
        self.path = None
        self.path_computed = False #Flag 
    
    def get_path(self):
        self.get_logger().info(f'Start Pose: {self.start_pose.position.x}, {self.start_pose.position.y}, {self.start_pose.position.z}')
        self.get_logger().info(f'Goal Pose: {self.end_pose.position.x}, {self.end_pose.position.y}, {self.end_pose.position.z}')

        # Wait for the action server to be ready
        self.action_client.wait_for_server()

        goal_msg = ComputePathToPose.Goal()

        startPoseStamped = PoseStamped()
        startPoseStamped.header.frame_id = "map"
        startPoseStamped.header.stamp = self.get_clock().now().to_msg()
        startPoseStamped.pose = self.start_pose
        goalPoseStamped = PoseStamped()
        goalPoseStamped.header.frame_id = "map"
        goalPoseStamped.header.stamp = self.get_clock().now().to_msg()
        goalPoseStamped.pose = self.end_pose

        goal_msg.start = startPoseStamped
        goal_msg.goal = goalPoseStamped

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)

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
        status = future.result().status
        result = future.result().result
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Path planning failed with status code: {status}')
            return None

        self.get_logger().info('Planning Done')
        self.path = result.path.poses
        self.path_computed = True


class Delivery(Node):
    def __init__(self):
        super().__init__('delivery') 

        # Subscription to Arucos ID to visit (launched by PathOfDelivering)
        self.subcription_arucos_to_visit = self.create_subscription(
            PoseArray,
            '/ordered_aruco_poses',
            self.delivery_location_callback,
            10)
        self.subcription_arucos_to_visit

        # Deliver pizza every 1 sec
        self.pizza_delivery = self.create_timer(1, self.pizza_delivery_callback)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.received_poses = False # Delivery after receing pose
        self.sent_goal = False # One goal at a time
        self.client_idx = 0 # Idx of pizza to deliver
        self.nb_clients = 0

    def delivery_location_callback(self, msg):
        self.poses_to_visit = msg.poses  #Pose[]
        self.nb_clients = len(self.poses_to_visit)
        self.received_poses = True
        self.destroy_subscription(self.subcription_arucos_to_visit)
    
    def pizza_delivery_callback(self):
        if not self.received_poses:
            return
        if self.sent_goal == False and self.client_idx < self.nb_clients:
            self.sent_goal = True
            self.get_logger().info('Sending goal to the robot')
            self.send_goal(self.poses_to_visit[self.client_idx])

        if self.client_idx >= self.nb_clients :
            self.get_logger().info('Delivery finished !')

        return

    def send_goal(self, pose):
        self.get_logger().info('Waiting for action server..')
        self._action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = pose 
        goal_msg.behavior_tree = ""

        self.get_logger().info('Sending goal request..')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted :
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succedeed !')
            self.client_idx +=1
        else:
            self.get_logger().info('Goal failed')
        
        self.sent_goal  = False
            
def main(args=None):
    rclpy.init(args=args)
    path_computer_node = PathOfDelivery()
    rclpy.spin(path_computer_node)
	
if __name__ == '__main__':
	main()