import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2


class image_publisher(Node):
    def __init__(self):
        super().__init__('image_publisher_node')

        self.bridge = CvBridge()
        self.image_publisher_ = self.create_publisher(Image, 'camera/raw_image', 10)
        self.pipeline = dai.Pipeline()
        
        cam = self.pipeline.create(dai.node.Camera).build()
        self.videoQueue = cam.requestOutput((640,400)).createOutputQueue()
        self.pipeline.start()

         # Timer callback at 30fps
        self.timer = self.create_timer(0.5, self.timer_callback)
    def timer_callback(self):
        if not self.pipeline.isRunning():
            self.get_logger().warn("Pipeline not running")
            return
        
        videoIn = self.videoQueue.get()
        if videoIn is None:
            return
        assert isinstance(videoIn, dai.ImgFrame)
        frame = videoIn.getCvFrame()
            
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera_link"
        self.image_publisher_.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = image_publisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
