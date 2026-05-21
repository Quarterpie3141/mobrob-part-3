import rclpy
from rclpy.node import Node
import time
from datetime import datetime
import subprocess
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

class EstopNode(Node):

    def __init__(self):
        super().__init__('estop_node')

        self.scan_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.status_sub = self.create_subscription(String,'/master/status',self.status_callback,10)

        self.estop_pub = self.create_publisher(Bool,'/Estop',10)
        self.timer = self.create_timer(0.1,self.publish_estop_state)

        # Parameters
        self.distance_threshold = 1.0
        self.required_hits = 5

        # State variables
        self.consecutive_hits = 0
        self.estop_active = False
        self.bag_recorded = False

        self.get_logger().info('E-Stop node started')

    def publish_estop_state(self):

        msg = Bool()
        msg.data = self.estop_active

        self.estop_pub.publish(msg)
    def scan_callback(self, msg: LaserScan):

        obstacle_detected = False

        # Check all valid ranges
        for r in msg.ranges:
            # Ignore invalid values
            if r == float('inf') or r != r:
                continue
            if r < self.distance_threshold:
                obstacle_detected = True
                break

        # Count consecutive detections
        if obstacle_detected:
            self.consecutive_hits += 1
            self.get_logger().info(
                f'Obstacle detected ({self.consecutive_hits}/{self.required_hits})'
            )
        else:
            self.consecutive_hits = 0

        # Trigger Estop after 5 consecutive detections
        if (self.consecutive_hits >= self.required_hits and not self.estop_active):
            self.estop_active = True

            estop_msg = Bool()
            estop_msg.data = True
            self.get_logger().warn('E-STOP ACTIVATED')
            if self.bag_recorded == False:
                self.bag_recorded = True
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                bag_path = f'/opt/ros2ws/img/estop_bag_{timestamp}'
                self.bag_process = subprocess.Popen(
                    [
                        'ros2',
                        'bag',
                        'record',
                        '-a',
                        '-o',
                        bag_path
                    ]
                )

                time.sleep(5)

                self.bag_process.terminate()


    def status_callback(self, msg: String):

        status = msg.data.lower().strip()

        if status == "clear estop":
            self.estop_active = False
            self.bag_recorded = False
            self.consecutive_hits = 0
            estop_msg = Bool()
            estop_msg.data = False

            self.estop_pub.publish(estop_msg)
            self.get_logger().info('E-STOP CLEARED')


def main(args=None):

    rclpy.init(args=args)
    node = EstopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()