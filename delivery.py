from geometry_msgs.msg import Pose, Transform, PoseArray
from tf2_geometry_msgs import do_transform_pose

def PoseToVisitForGivenAruco(ArucoPose):
    #What is the orientation in the ArucoPose ?
    
    t1 = Transform() #UpperLeftCornerTransform

    # This transform represents a shift from (0, 0) to (0, 30)
    t1.translation.x = 0.0
    t1.translation.y = 0.30
    t1.translation.z = 0.0

    # Rotation of the Pose should be confirmed using TurtleBot
    # We want to get a vector that face the wall
    t1.rotation.x = 0.0
    t1.rotation.y = 0.0
    t1.rotation.z = 3.1415 
    t1.rotation.w = 1.0 #What is w?
    
    return do_transform_pose(ArucoPose, t1)