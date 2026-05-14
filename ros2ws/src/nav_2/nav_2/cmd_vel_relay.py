import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscriber: Listens to the output of the smoother
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_smoothed',
            self.listener_callback,
            10)
        
        # Publisher: Sends it to the final motor command topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('CmdVel Relay Node started. Listening to cmd_vel_smoothed...')

    def listener_callback(self, msg):
        # Simply take the received message and publish it to the new topic
        self.publisher_.publish(msg)
        # Log to the console so you can see it working in real-time
        self.get_logger().info(f'Relaying: Linear X: {msg.linear.x:.2f}, Angular Z: {msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_relay = CmdVelRelay()
    rclpy.spin(cmd_vel_relay)
    cmd_vel_relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()