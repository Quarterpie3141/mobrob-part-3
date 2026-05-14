import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2
import ultralytics
from inference import get_model


class DepthAICameraNode(Node):
    def __init__(self):
        super().__init__('depthai_camera_node')
        self.model = get_model(
        model_id="test-2-150-per/1",
        api_key="Kms3xRqZ4JstX4UopFEA"
        )

        self.CLASSES = {'- negative': "alpha", '1': "beta", '2': "delta", '3': "eta", '4': "lambda", '5': "mu", '6': "rho", '7': "tau", '8': "psi", '9': "omega"}

        self.bridge = CvBridge()
        self.letter_detection_pub = self.create_publisher(String, 'camera/letter_detection', 10)

        self.raw_image = self.create_subscription(Image, 'camera/raw_image', self.raw_image_callback, 10)


    def raw_image_callback(self, msg):
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        half = 290
        frame = frame[cy - half:cy + half, cx - half:cx + half]

        results = self.model.infer(frame)[0]

        # Draw detections
        for pred in results.predictions:
            x1 = int(pred.x - pred.width / 2)
            y1 = int(pred.y - pred.height / 2)
            x2 = int(pred.x + pred.width / 2)
            y2 = int(pred.y + pred.height / 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"{self.CLASSES[pred.class_name]} {pred.confidence:.0%}",
                (x1, y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )
            
            predicted_letter = self.CLASSES[pred.class_name]
            self.letter_detection_pub.publish(predicted_letter)

      

def main(args=None):
    rclpy.init(args=args)
    node = DepthAICameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
