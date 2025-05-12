import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Pose, Transform, PoseArray
from tf2_geometry_msgs import do_transform_pose
from mapping import convert_Pose_to_ij #TO DO

class wallsObservation(Node):
    def __init__(self):
        super().__init__('walls_observation')
        self.subscription = self.create_subscription(
            Pose, 
            'aruco_markers', #To which topic do I have to listen to to get the position of the camera
            self.walls_callback,
            10)
        self.subscription
        self.publisher = self.create_publisher(PoseArray, 'walls_observed', 10)
    
    def walls_callback(self, robotPose):
        # It will compute the pose of the robot, the pose of the upper left and upper right corner of a rectangle
        t1 = Transform() #UpperLeftCornerTransform

        # This transform represents a shift from (0, 0) to (15, 30)
        # Get better values using robot should be done
        t1.translation.x = 0.15
        t1.translation.y = 0.30
        t1.translation.z = 0.0

        # No rotation
        t1.rotation.x = 0.0
        t1.rotation.y = 0.0
        t1.rotation.z = 0.0
        t1.rotation.w = 1.0 #What is w?
        
        t2 = Transform() #UpperRightCornerTransform

        # This transform represents a shift from (0, 0) to (-15, 30)
        # Get better values using robot should be done
        t2.translation.x = -0.15
        t2.translation.y = 0.30
        t2.translation.z = 0.0

        # No rotation
        t2.rotation.x = 0.0
        t2.rotation.y = 0.0
        t2.rotation.z = 0.0
        t2.rotation.w = 1.0
        
        msg = PoseArray()
        msg.append(robotPose)
        msg.append(do_transform_pose(robotPose, t1))
        msg.append(do_transform_pose(robotPose, t2))
        self.publisher_.publish(msg)


def updateMapping(PoseArray):
    robot = convert_Pose_to_ij(PoseArray[0])
    URcorner = convert_Pose_to_ij(PoseArray[1])
    ULcorner = convert_Pose_to_ij(PoseArray[2])
    # We need to know if index (i,j) is seen by the camera or not
    
    #Loop over all (i,j) of the map => How ?
    i = 0
    j = 0
    if(isSeenByCamera(i,j,robot, URcorner, ULcorner)):
        #We have to check that it's a wall
        if(map[i,j] == wall): #How?
            map[i,j] == Observed

def isSeenByCamera(i,j, robot, URcorner, ULcorner):
    # Return false if (i,j) is not in the camera field of vision, true if yes
    distance = 10**2 + 5**2
    if(EuclideanDistance(i,j, robot[0], robot[1]) > distance): #Which value to consider => value in map discretization (not in cm !)
        return False
    if(EuclideanDistance(i,j, URcorner[0],URcorner[1]) > distance):
        return False
    if(EuclideanDistance(i,j, ULcorner[0], ULcorner[1]) > distance):
       return False
    return True

def EuclideanDistance(i1,j1, i2,j2): #TBC see prove of Concept
    return (i1-i2)**2 + (j1-j2)**2
        