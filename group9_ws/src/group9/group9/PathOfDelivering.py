import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray
import planning

class PathOfDelivering(Node):

    def __init__(self):
        super().__init__('path_of_delivering')  
        self.subscriptionLocation = self.create_subscription(
            PoseArray,
            'robot_location',
            self.fullLocation,
            10)
        self.subcriptionIds = self.create_subcription(
            Int32MultiArray,
            'delivery_locations',
            self.getIDs,
            10)
        self.startingRobotLocation #How to know the robot location at which the path must start?
        self.subscription
        self.publisher = self.create_publisher(PoseArray, 'ordered_poses', 10)
        self.fullPose= [None]
        self.poseToVisit = [None]

    def fullLocation(self, msg):
        self.fullPose = msg.poses
        self.destroy_subscription(self.subscriptionLocation)
    
    def getIDs(self, msg):
        if all(pose is not None for pose in self.fullPose):
            id = msg.data
            self.nbArucoToVisit = len(id)
            for i in range(0,len(self.fullPose)):
                if i in id:
                    self.poseToVisit.append(self.fullPose[i])
            self.destroy_subscription(self.subcriptionIds)
            self.computation(self)
    
    def computation(self):
        distances = planning(self.startingRobotLocation, self.poseToVisit) 
            
def main(args=None):
	rclpy.init(args=args)
	deliver_node = PathOfDelivering()
	rclpy.spin(deliver_node)
	deliver_node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()

        
        




