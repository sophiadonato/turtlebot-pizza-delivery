import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Making360(Node):

    def __init__(self):
        super().__init__('making_360')
        self.publisher = self.create_publisher(
			Twist, 
			'/cmd_vel',  #command velocity -> to check
			10)
        self.publisher
        self.angular_speed = 0.3 #rad/s
    
    def rotate(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed
        
        angle = 2*4 #more or less 360 degrees
        duration = angle/self.angular_speed
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info("Starting rotation")

        while rclpy.ok():
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time-start_time >= duration:
                break
            self.publisher.publish(twist)
            time.sleep(0.1)
        
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Finished 360")

def main(args=None):
    rclpy.init(args=args)
    rotating_node = Making360()
    rotating_node.rotate()
    rotating_node.destroy_node()
    rclpy.shutdown()
	
if __name__ == '__main__':
	main()