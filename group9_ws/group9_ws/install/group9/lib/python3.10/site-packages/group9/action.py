import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class NavigateToPoseActionClient(Node):
    
    def __init__(self):
        super().__init__("navigate_to_pose_action_client")
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    
    def send_goal(self, x, y):
        self.get_logger().info('Waiting for action server..')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map" #-> move wrt to this frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x #In meters !
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.behavior_tree =""

        self.get_logger().info('Sending goal request..')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

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
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED :
            self.get_logger().info('Goal succeeded !')
        else :
            self.get_logger().info('Goal failed.')
        
        rclpy.shutdown()


def main():
    rclpy.init()
    action_client = NavigateToPoseActionClient()
    action_client.send_goal(1.0, 1.0)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

            


