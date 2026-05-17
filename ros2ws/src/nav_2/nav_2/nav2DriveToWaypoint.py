import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class NavToPointClient(Node):

    def __init__(self):
        super().__init__('nav_to_point_client')
        
        # Client for nav2 server
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('Waiting for Nav2 "navigate_to_pose" action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server detected. Ready for commands.')

    def send_goal(self, x, y, theta_degrees):
        goal_msg = NavigateToPose.Goal()

        #pose
        pose = PoseStamped()
        pose.header.frame_id = 'map'  
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        heading_rad = math.radians(theta_degrees)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(heading_rad / 2.0)
        pose.pose.orientation.w = math.cos(heading_rad / 2.0)

        goal_msg.pose = pose


        #LOGGER  
        self.get_logger().info(f'Sending goal: X={x}, Y={y}, Heading={theta_degrees}° to Nav2...')
        
        #send msg
        self._action_client.send_goal_async(goal_msg) 



def main(args=None):
    rclpy.init(args=args)
    node = NavToPointClient()

    target_x = 0
    target_y = 5
    target_heading = 90.0  

    node.send_goal(target_x, target_y, target_heading)
    rclpy.spin(node)


if __name__ == '__main__':
    main()