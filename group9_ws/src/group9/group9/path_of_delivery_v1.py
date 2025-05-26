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

        # Action client to compute path to pose
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.computing_distance = False

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
                distance_matrix[i][j] = self.computeEstimatedDistance(poses[i], poses[j]) #Check indexes
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
        self.getPath(start,end)
        
        while True:
            if self.computing_distance == False:
                break
        distance = 0.0
        self.get_logger().info(f'before distance computed')
        for i in range(1, len(self.path_poses)):
            x0 = self.path_poses[i - 1].pose.position.x
            y0 = self.path_poses[i - 1].pose.position.y
            x1 = self.path_poses[i].pose.position.x
            y1 = self.path_poses[i].pose.position.y
            distance += math.hypot(x1 - x0, y1 - y0)
        self.get_logger().info(f'distance computed')
        return distance
    
    def getPath(self, start, goal):
        self.computing_distance = True
        self.get_logger().info(f'Start Pose: {start.position.x}, {start.position.y}, {start.position.z}')
        self.get_logger().info(f'Goal Pose: {goal.position.x}, {goal.position.y}, {goal.position.z}')

        # Wait for the action server to be ready
        self.compute_path_to_pose_client.wait_for_server()

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
        self._send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted():
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
        self.computing_distance = False
        self.path_poses = result.result.path.poses


            
def main(args=None):
	rclpy.init(args=args)
	deliver_node = PathOfDelivering()
	rclpy.spin(deliver_node)
	deliver_node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()