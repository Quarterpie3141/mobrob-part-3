import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscriber: Listens to the output of the smoother
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_smoothed',
            self.listener_callback,
            10)
        self.current_joy = None
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.smoothed_cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel_smoothed', 
            self.listener_callback, 
            10)
        self.behavior_cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel_behavior_server',
            self.listener_callback,
            10)
        self.controller_cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel_controller_server',
            self.listener_callback,
            10)
        

        # Publisher: Sends it to the final motor command topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('CmdVel Relay Node started. Listening to cmd_vel_smoothed...')

        self.estop_subscription = self.create_subscription(Bool, '/estop_button', self.estop_callback, 10)
        self.estopped = False

    def joy_callback(self, msg):
        self.current_joy = msg
    def estop_callback(self, msg):
        self.estopped = msg.data

    def listener_callback(self, msg):

        self.publisher_.publish(msg)
        self.get_logger().info(f'Relaying: Linear X: {msg.linear.x:.2f}, Angular Z: {msg.angular.z:.2f}')


        # Simply take the received message and publish it to the new topic
        if self.estopped:
            command = Twist()
            command.linear.x = 0.0
            command.linear.y = 0.0
            command.linear.z = 0.0
            command.angular.x = 0.0
            command.angular.y = 0.0
            command.angular.z = 0.0
            self.publisher_.publish(command)

        elif (self.current_joy is not None) and self.current_joy.buttons[9] == 1 and self.current_joy.buttons[10] == 1:
        
            self.publisher_.publish(msg)
            self.get_logger().info(f'Relaying: Linear X: {msg.linear.x:.2f}, Angular Z: {msg.angular.z:.2f}')
        else:
            command = Twist()
            command.linear.x = 0.0
            command.linear.y = 0.0
            command.linear.z = 0.0
            command.angular.x = 0.0
            command.angular.y = 0.0
            command.angular.z = 0.0
            self.publisher_.publish(command)


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_relay = CmdVelRelay()
    rclpy.spin(cmd_vel_relay)
    cmd_vel_relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()