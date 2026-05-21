import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from inference import get_model
from std_msgs.msg import String
import cv2



class greek(Node):
    def __init__(self):
        super().__init__('greek')
        self.model = get_model(
        model_id="test-2-150-per/5",
        api_key="Kms3xRqZ4JstX4UopFEA"
        )
        self.frame = None
        self.bridge = CvBridge()
        self.letter_detection_pub = self.create_publisher(String, 'camera/letter_detection', 10)

        self.raw_image = self.create_subscription(Image, 'camera/raw_image', self.raw_image_callback, 10)

        #Check image command
        self.start_letter_detection_sub = self.create_subscription(String, '/check_label', self.start_letter_detection_callback, 10)

    def start_letter_detection_callback(self, msg):
        if msg.data == "classify":

            if self.frame is None:
                self.letter_detection_pub.publish(String(data="no frame available"))
                return

            results = self.model.infer(self.frame)[0]
            
            if not results.predictions:
                self.letter_detection_pub.publish(String(data="nothing detected"))
                return
            
            pred = max(results.predictions, key=lambda p: p.confidence)
                
            if pred.confidence > 0.8:
                self.letter_detection_pub.publish(String(data=pred.class_name))
                
            else:
                self.letter_detection_pub.publish(String(data="confidence too low"))



    def raw_image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        
def main(args=None):
    rclpy.init(args=args)
    node = greek()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
