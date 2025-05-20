import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray
from python_tsp.exact import solve_tsp_dynamic_programming #pip install python-tsp
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose

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

    def fullLocation(self, msg):
        self.fullPose = msg.poses
        self.destroy_subscription(self.subscriptionLocation)
    
    def getIDs(self, msg):
        if all(pose is not None for pose in self.fullPose):
            self.id = msg.data
            self.nbArucoToVisit = len(self.id)
            for i in range(0,len(self.fullPose)):
                if i in self.id:
                    np.append(self.poseToVisit,np.array([self.fullPose[i]]))
            self.destroy_subscription(self.subcriptionIds)
            self.computation()
    
    def computation(self):
        distance_matrix = np.zeros((self.nbArucoToVisit + 1,self.nbArucoToVisit + 1))
        robotLocation = np.array(self.startingRobotLocation)
        poses = np.append(robotLocation,self.poseToVisit)
        get_logger(f'poses size {poses.size}')
        for i in range(0,self.nbArucoToVisit +1): 
            for j in range(i + 1, self.nbArucoToVisit + 1):
                distance_matrix[i][j] = self.computeEstimatedDistance(poses[i], poses[j]) #Check indexes
                distance_matrix[j][i] = distance_matrix[i][j]
        distance_matrix[:,0] = 0
        permutation,_ = solve_tsp_dynamic_programming(distance_matrix)
        permutation = permutation[1:] #The first element is always the current location of the Robot
        sequence = PoseArray
        for nb in permutation:
            sequence.append(self.PoseToVisit(nb -1))
        self.publisher.publish(sequence)
    
    def computeEstimatedDistance(self, start, end):
        path = None
        while path is None:
            path = self.getPath(start,end)
        distance = 0.0
        for i in range(1, len(path)):
            x0 = path[i - 1].pose.position.x
            y0 = path[i - 1].pose.position.y
            x1 = path[i].pose.position.x
            y1 = path[i].pose.position.y
            distance += math.hypot(x1 - x0, y1 - y0)
        return distance
    
    def getPath(self, start, goal):
        # Sends a `NavToPose` action request
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal
        goal_msg.start = start

        send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn('Getting path failed with status code: {0}'.format(self.status))
            return None

        return self.result_future.result().result.path
        
            
def main(args=None):
	rclpy.init(args=args)
	deliver_node = PathOfDelivering()
	rclpy.spin(deliver_node)
	deliver_node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()