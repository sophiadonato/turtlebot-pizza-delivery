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
        self.subscriptionLocation = self.create_subscription(
            PoseArray,
            'robot_location',
            self.fullLocation,
            10)
        self.subcriptionIds = self.create_subscription(
            Int32MultiArray,
            'delivery_locations',
            self.getIDs,
            10)
        self.startingRobotLocation = Pose() #How to know the robot location at which the path must start? -> pose of robot done in our code
        self.publisher = self.create_publisher(PoseArray, 'ordered_poses', 10)
        self.fullPose= [None]
        self.poseToVisit = np.array([]) #init good ? 
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

    def fullLocation(self, msg):
        self.fullPose = msg.poses
        self.destroy_subscription(self.subscriptionLocation)
    
    def getIDs(self, msg):
        if all(pose is not None for pose in self.fullPose):
            self.id = msg.data
            self.nbArucoToVisit = len(self.id)
            for i in range(0,len(self.fullPose)):
                if i in self.id:
                    self.poseToVisit = np.append(self.poseToVisit,np.array([self.fullPose[i]]))
            self.destroy_subscription(self.subcriptionIds)
            self.computation()
    
    def computation(self):
        distance_matrix = np.zeros((self.nbArucoToVisit + 1,self.nbArucoToVisit + 1))
        robotLocation = np.array(self.startingRobotLocation)
        poses = np.append(robotLocation,self.poseToVisit)
        for i in range(0,self.nbArucoToVisit +1): 
            for j in range(i + 1, self.nbArucoToVisit + 1):
                self.get_logger().info(f'Before computation Distance')
                #distance_matrix[i][j] = self.computeEstimatedDistance(poses[i], poses[j]) #Check indexes
                distance_matrix[i][j] = i*j*2.5
                distance_matrix[j][i] = distance_matrix[i][j]
        distance_matrix[:,0] = 0
        self.get_logger().info(f'BEFORE TSP')
        permutation,_ = solve_tsp_dynamic_programming(distance_matrix)
        self.get_logger().info(f'{permutation}')
        permutation = permutation[1:] #The first element is always the current location of the Robot
        sequence = PoseArray()
        for nb in permutation:
            sequence.poses.append(self.poseToVisit[nb -1])
        self.publisher.publish(sequence)
    
    def computeEstimatedDistance(self, start, end):
        path = None
        while path is None:
            path = self.getPath(start,end)
        distance = 0.0
        self.get_logger().info(f'before distance computed')
        for i in range(1, len(path)):
            x0 = path[i - 1].pose.position.x
            y0 = path[i - 1].pose.position.y
            x1 = path[i].pose.position.x
            y1 = path[i].pose.position.y
            distance += math.hypot(x1 - x0, y1 - y0)
        self.get_logger().info(f'distance computed')
        return distance
    
    def getPath(self, start, goal):
        self.get_logger().info(f'Start Pose: {start.position.x}, {start.position.y}, {start.position.z}')
        self.get_logger().info(f'Goal Pose: {goal.position.x}, {goal.position.y}, {goal.position.z}')

        # Wait for the action server to be ready
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Waiting for 'compute_path_to_pose' action server...")
        
        self.get_logger().info("Action server is ready!")

        goal_msg = ComputePathToPose.Goal()

        startPoseStamped = PoseStamped()
        startPoseStamped.pose.position.x = start.position.x
        startPoseStamped.pose.position.y = start.position.y
        startPoseStamped.pose.position.z = start.position.z

        goalPoseStamped = PoseStamped()
        goalPoseStamped.pose.position.x = goal.position.x
        goalPoseStamped.pose.position.y = goal.position.y
        goalPoseStamped.pose.position.z = goal.position.z

        goal_msg.start = startPoseStamped
        goal_msg.goal = goalPoseStamped

        self.get_logger().info('Sending goal...')
        try:
            send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal_msg)
            #rclpy.spin_until_future_complete(self, send_goal_future)
        except Exception as e:
            self.get_logger().error(f"Error sending goal: {e}")
            return None

        # Log if goal was accepted
        goal_handle = send_goal_future.result()
        self.get_logger().info(f'Goal handle: {goal_handle}')
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal was rejected by action server. Status: {goal_handle.status}')
            return None

        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        #rclpy.spin_until_future_complete(self, get_result_future)

        # Log result
        result = get_result_future.result()
        self.get_logger().info(f'Action result: {result}')

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Path planning failed with status code: {result.status}')
            return None

        self.get_logger().info('Planning Done')
        return result.result.path.poses


            
def main(args=None):
	rclpy.init(args=args)
	deliver_node = PathOfDelivering()
	rclpy.spin(deliver_node)
	deliver_node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()