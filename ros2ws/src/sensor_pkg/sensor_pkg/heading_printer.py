import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def quaternion_to_yaw_degrees(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_radians = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_radians)


class HeadingPrinter(Node):
    def __init__(self):
        super().__init__('heading_printer')

        self.create_subscription(Imu, 'imu/data_raw', self.imu_data_raw_callback, 10)
        self.create_subscription(Imu, 'imu/data', self.imu_data_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odometry_callback, 10)
        self.imu_comp_publisher = self.create_publisher(Float64, 'imu/comp', 10)

    def imu_data_raw_callback(self, msg):
        self._print_heading('imu/data_raw', msg.orientation, msg.orientation_covariance)

    def imu_data_callback(self, msg):
        heading = self._print_heading('imu/data', msg.orientation, msg.orientation_covariance)
        if heading is None:
            return

        heading_msg = Float64()
        heading_msg.data = heading
        self.imu_comp_publisher.publish(heading_msg)

    def odometry_callback(self, msg):
        self._print_heading('/odometery/filtered', msg.pose.pose.orientation)

    def _print_heading(self, topic_name, orientation, orientation_covariance=None):
        if self._orientation_unavailable(orientation, orientation_covariance):
            return None

        heading = quaternion_to_yaw_degrees(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        return heading

    @staticmethod
    def _orientation_unavailable(orientation, orientation_covariance):
        zero_quaternion = (
            orientation.x == 0.0
            and orientation.y == 0.0
            and orientation.z == 0.0
            and orientation.w == 0.0
        )
        invalid_covariance = (
            orientation_covariance is not None and len(orientation_covariance) > 0 and orientation_covariance[0] < 0.0
        )
        return zero_quaternion or invalid_covariance


def main(args=None):
    rclpy.init(args=args)
    node = HeadingPrinter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()