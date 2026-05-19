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
            
            results = self.model.infer(self.frame)[0]

            if not results.predictions:
                self.letter_detection_pub.publish(String(data="nothing detected"))
                return
               
            
        # Draw detections
        for pred in results.predictions:
            if pred.confidence > 0.5:
                predicted_letter = pred.class_name
                self.letter_detection_pub.publish(String(data=predicted_letter))
                self.letter_detection_pub.publish(String(data=str(pred.confidence)))

            else:
                self.letter_detection_pub.publish(String(data="confidece too low"))
                self.letter_detection_pub.publish(String(data=str(pred.confidence)))



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
