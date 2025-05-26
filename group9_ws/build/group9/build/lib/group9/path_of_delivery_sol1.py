# Solution with a separate node for the computation of the path (avoid blocking problems)
import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray,PoseStamped
from std_msgs.msg import Int32MultiArray
from python_tsp.exact import solve_tsp_dynamic_programming #pip install python-tsp
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

class PathOfDelivering(Node):

    def __init__(self):
        super().__init__('path_of_delivering') 

        # Subscription to Robot pose
        self.subscription_robot_pose = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.robot_pose_callback,
            10)
        self.subscription_robot_pose
        self.robot_pose = None


        # Subscription to Arucos locations (why robot_location? ask Arnaud) 
        self.subscription_arucos_location = self.create_subscription(
            PoseArray,
            'arucos_location',
            self.full_location,
            10)
        self.subscription_arucos_location

        # Subscription to Arucos ID to visit (launched by profs)
        self.subcription_IDs = self.create_subscription(
            Int32MultiArray,
            'delivery_locations',
            self.get_IDs,
            10)
        self.subcription_IDs
        

        # Publisher of ordered Arucos location
        self.publisher = self.create_publisher(PoseArray, 'ordered_poses', 10)
        self.full_pose= [None] #init good?
        self.pose_to_visit = np.array([]) #init good ? 

        # Action client node to compute path to pose
        self.compute_path_node = ComputePathBetweenLocations()

    def full_location(self, msg):
        self.full_pose = msg.poses
        self.destroy_subscription(self.subscription_arucos_location)

    def robot_pose_callback(self, msg):
        self.robot_pose = msg.pose
        self.destroy_subscription(self.subscription_robot_pose)
    
    def get_IDs(self, msg):
        if all(pose is not None for pose in self.full_pose) and self.robot_pose != None:
            self.id = msg.data
            self.nb_aruco_to_visit = len(self.id)
            for i in range(0,len(self.full_pose)):
                if i in self.id:
                    self.pose_to_visit = np.append(self.pose_to_visit,np.array([self.full_pose[i]]))
            self.destroy_subscription(self.subcription_IDs)
            self.computation()
    
    def computation(self):
        distance_matrix = np.zeros((self.nb_aruco_to_visit + 1,self.nb_aruco_to_visit + 1))
        robot_location = np.array(self.robot_pose)
        poses = np.append(robot_location,self.pose_to_visit)
        self.get_logger().info(f'poses :{poses}')
        for i in range(0,self.nb_aruco_to_visit +1): 
            for j in range(i + 1, self.nb_aruco_to_visit + 1):
                distance_matrix[i][j] = self.compute_estimated_distance(poses[i], poses[j]) #Check indexes
                distance_matrix[i][j] = i*j*2.5
                distance_matrix[j][i] = distance_matrix[i][j]
        distance_matrix[:,0] = 0
        self.get_logger().info(f'Computing delibery order using TSP')
        permutation,_ = solve_tsp_dynamic_programming(distance_matrix)
        self.get_logger().info(f'{permutation}')
        permutation = permutation[1:] #The first element is always the current location of the Robot
        sequence = PoseArray()
        for nb in permutation:
            sequence.poses.append(self.pose_to_visit[nb -1])
        self.publisher.publish(sequence)
    
    def compute_estimated_distance(self, start, end):
        self.compute_path_node.start_pose = start
        self.compute_path_node.end_pose = end
        self.compute_path_node.path_computed = False

        self.get_logger().info(f'Compute path using Action Client node')
        self.compute_path_node.get_path()
        while self.compute_path_node.path_computed == False:
            self.get_logger().info('In loop')
            rclpy.spin_once(self.compute_path_node, timeout_sec = 3)


        distance = 0.0
        self.get_logger().info(f'Compute estimated distance')
        for i in range(1, len(self.compute_path_node.path)):
            x0 = self.compute_path_node.path[i - 1].pose.position.x
            y0 = self.compute_path_node.path[i - 1].pose.position.y
            x1 = self.compute_path_node.path[i].pose.position.x
            y1 = self.compute_path_node.path[i].pose.position.y
            distance += math.hypot(x1 - x0, y1 - y0)
        self.get_logger().info(f'distance computed')
        return distance
    
class ComputePathBetweenLocations(Node):
    def __init__(self):
        super().__init__('compute_path_btw_locations')
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.start_pose = None
        self.end_pose = None
        self.path = None
        self.path_computed = False
    
    def get_path(self):
        self.get_logger().info(f'Start Pose: {self.start_pose.position.x}, {self.start_pose.position.y}, {self.start_pose.position.z}')
        self.get_logger().info(f'Goal Pose: {self.end_pose.position.x}, {self.end_pose.position.y}, {self.end_pose.position.z}')

        # Wait for the action server to be ready
        self.action_client.wait_for_server()

        goal_msg = ComputePathToPose.Goal()

        startPoseStamped = PoseStamped()
        startPoseStamped.pose.position.x = self.start_pose.position.x
        startPoseStamped.pose.position.y = self.start_pose.position.y
        startPoseStamped.pose.position.z = self.start_pose.position.z

        goalPoseStamped = PoseStamped()
        goalPoseStamped.pose.position.x = self.end_pose.position.x
        goalPoseStamped.pose.position.y = self.end_pose.position.y
        goalPoseStamped.pose.position.z = self.end_pose.position.z

        goal_msg.start = startPoseStamped
        goal_msg.goal = goalPoseStamped

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.get_logger().info('Goal has been send...')

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.get_logger().info('In response callback ')
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
            self.path = None
            self.path_computed = True
            return None

        self.get_logger().info('Planning Done')
        self.path = result.result.path.poses
        self.path_computed = True

            
def main(args=None):
    rclpy.init(args=args)
    deliver_node = PathOfDelivering()
    rclpy.spin(deliver_node)
    deliver_node.destroy_node()
    rclpy.shutdown()
	
if __name__ == '__main__':
	main()