import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray

class Fake(Node):

    def __init__(self):
        super().__init__('fake')  
        self.publisher1 = self.create_publisher(PoseArray, '/arucos_location', 10)
        self.publisher2 = self.create_publisher(Int32MultiArray, '/delivery_locations', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info("Computing poses of arucos")
        Ap1 = PoseArray()
        p1 = Pose()
        p1.position.x = -0.6
        p1.position.y = -1.4
        p1.position.z = 0.0
        Ap1.poses.append(p1)
        p2 = Pose()
        p2.position.x = 1.4
        p2.position.y = -1.9
        p2.position.z = 0.0
        Ap1.poses.append(p2)

        p3 = Pose()
        p3.position.x = -1.0
        p3.position.y = -2.0
        p3.position.z = 0.0
        Ap1.poses.append(p3)

        self.publisher1.publish(Ap1)
        v = Int32MultiArray()
        v.data.append(0)
        v.data.append(1)
        self.publisher2.publish(v)
    

def main(args=None):
	rclpy.init(args=args)
	deliver_node = Fake()
	rclpy.spin(deliver_node)
	deliver_node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()