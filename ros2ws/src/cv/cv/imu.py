#!/usr/bin/env python3
"""Bare-bones ROS2 Humble node that publishes IMU data from a DepthAI camera (v3 API)."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import depthai as dai


class DepthAIImuNode(Node):
    def __init__(self):
        super().__init__("depthai_imu_node")

        # ROS2 publisher
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)

        # DepthAI v3 pipeline
        self.pipeline = dai.Pipeline()

        imu = self.pipeline.create(dai.node.IMU)

        # Enable accelerometer and gyroscope at 400 Hz
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 400)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        # v3 API: create output queue directly from the node output
        self.imu_queue = imu.out.createOutputQueue(maxSize=50, blocking=False)

        self.pipeline.start()

        # Timer to poll the queue
        self.timer = self.create_timer(1.0 / 200.0, self.timer_callback)
        self.get_logger().info("DepthAI IMU node started")

    def timer_callback(self):
        imu_data = self.imu_queue.tryGet()
        if imu_data is None:
            return

        for packet in imu_data.packets:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "oak_imu_frame"

            # Accelerometer
            accel = packet.acceleroMeter
            msg.linear_acceleration.x = float(accel.x)
            msg.linear_acceleration.y = float(accel.y)
            msg.linear_acceleration.z = float(accel.z)

            # Gyroscope
            gyro = packet.gyroscope
            msg.angular_velocity.x = float(gyro.x)
            msg.angular_velocity.y = float(gyro.y)
            msg.angular_velocity.z = float(gyro.z)

            #Orientation
            rot = packet.rotationVector
            msg.orientation.x = float(rot.i)
            msg.orientation.y = float(rot.j)
            msg.orientation.z = float(rot.k)
            msg.orientation.w = float(rot.real)

            self.imu_pub.publish(msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthAIImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()