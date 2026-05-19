from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import os
import math
import time
import rclpy
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import ultralytics


class DepthAICameraNode(Node):
    def __init__(self):
        super().__init__('depthai_camera_node')

        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, "camera/detections", 10)
        self.image_pub = self.create_publisher(Image, "camera/detections/image", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.slave_status_pub = self.create_publisher(String, '/slave/status', 10)
        self.object_info_pub = self.create_publisher(String, "/semantic_objects", 10)
        self.scan_complete_pub = self.create_publisher(Bool, '/scan_complete', 10)

        # Subscribers
        self.bridge = CvBridge()
        self.master_status_sub = self.create_subscription(String, '/master/status', self.master_status_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/Odom', self.odom_callback, 10)
        self.cam_sub = self.create_subscription(Image, 'camera/raw_image', self.cam_sub_callback, 10)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # State
        self.master_status = None
        self.last_master_status = None
        self.frame = None
        self.results = []
        self.detected_objects = set()
        self.position_threshold = 1.0  # meters - same class at new location counts as new detection
        self.framecount = 0

        # Robot pose and velocity
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.prev_yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.robot_moving = False

        # Directories
        self.imgdir = os.path.join(os.getcwd(), "img/")
        os.makedirs(self.imgdir, exist_ok=True)
        self.modeldir = os.path.join(os.getcwd(), "src", "cv", "models")

        # Detection tuning parameters
        self.target_locked_frames = 0
        self.required_locked_frames = 5
        self.center_threshold_px = 60
        self.min_box_area = 25000

        # Search state
        self.picture_taken = False
        self.searching = False
        self.search_start_yaw = 0.0
        self.accumulated_rotation = 0.0
        self.full_rotation_threshold = 2.0 * math.pi
        self.total_yaw = 0.0
        self.yaw_initialised = False

        # YOLO model
        weights = os.path.join(self.modeldir, "part3v2.pt")
        self.model = ultralytics.YOLO(weights)
        self.label_map = self.model.names

    def cam_sub_callback(self, msg):
        """
        Camera subscriber callback.

        - Stores latest camera frame
        - Runs YOLO inference every 3rd frame
        - Publishes Detection2DArray
        - Publishes semantic object information
        """
        # if not hasattr(self, 'framecount'):
        #     self.framecount = 0

        self.framecount += 1

        # only process every 3rd frame
        if self.framecount % 3 != 0:
            return

        self.frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        self.results = self.model.predict(
            self.frame,
            conf=0.7,
            verbose=False
        )

        if self.master_status == 'taking picture':
            return

        detection_array_msg = Detection2DArray()
        time_now = self.get_clock().now().to_msg()
        detection_array_msg.header.stamp = time_now
        detection_array_msg.header.frame_id = "camera_link"

        for result in self.results:

            if result.boxes is None:
                continue

            for box in result.boxes:
                x1, y1, x2, y2 = (
                    box.xyxy[0]
                    .cpu()
                    .numpy()
                    .astype(int)
                )

                width = x2 - x1
                height = y2 - y1

                # filter tiny detections
                if width * height < self.min_box_area:
                    continue

                detection = Detection2D()
                bbox = BoundingBox2D()

                center_x = float((x1 + x2) / 2.0)
                center_y = float((y1 + y2) / 2.0)
                bbox.center.position.x = center_x
                bbox.center.position.y = center_y
                bbox.size_x = float(width)
                bbox.size_y = float(height)
                detection.bbox = bbox

                cls = int(box.cls[0].cpu().numpy())

                classification = self.label_map.get(
                    cls,
                    str(cls)
                )

                confidence = float(
                    box.conf[0].cpu().numpy()
                )

                hypothesis = ObjectHypothesisWithPose()

                hypothesis.hypothesis.class_id = classification
                hypothesis.hypothesis.score = confidence
                detection.results.append(hypothesis)
                detection_array_msg.detections.append(detection)
                # publish message
                object_msg = String()

                object_msg.data = (
                    f"{classification},"
                    f"{center_x:.1f},"
                    f"{center_y:.1f},"
                    f"{width},"
                    f"{height},"
                    f"{confidence:.2f},"
                    f"{self.robot_x:.2f},"
                    f"{self.robot_y:.2f},"
                    f"{self.robot_yaw:.2f}"
                )

                # only publish new classes
                # only publish if this class hasn't been seen near this location
                rounded_x = round(self.robot_x / self.position_threshold) * self.position_threshold
                rounded_y = round(self.robot_y / self.position_threshold) * self.position_threshold
                detection_key = (classification, rounded_x, rounded_y)

                if detection_key not in self.detected_objects:
                    self.detected_objects.add(detection_key)
                    self.object_info_pub.publish(object_msg)
                    self.get_logger().info(f"Published semantic object: {object_msg.data}")

                cv2.rectangle(
                    self.frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    self.frame,
                    f"{classification}: {confidence:.2f}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

        # publish detection array
        self.detection_pub.publish(detection_array_msg)
        # publish annotated image
        detection_img_msg = self.bridge.cv2_to_imgmsg(
            self.frame,
            encoding="bgr8"
        )
        detection_img_msg.header.stamp = time_now
        detection_img_msg.header.frame_id = "camera_link"

        self.image_pub.publish(detection_img_msg)


    def master_status_callback(self, msg: String):
        self.last_master_status = getattr(self, 'master_status', None)
        self.master_status = msg.data

    def odom_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y


        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.omega = msg.twist.twist.angular.z

        self.robot_moving = abs(self.vx) > 0.01 or abs(self.omega) > 0.01

        if self.searching:

            if not self.yaw_initialised:
                self.prev_yaw = self.robot_yaw
                self.yaw_initialised = True

            delta = self.yaw_diff(self.robot_yaw, self.prev_yaw)

            if abs(delta) < 0.01:
                delta = 0.0

            self.total_yaw += abs(delta)
            self.prev_yaw = self.robot_yaw


    def yaw_diff(self, current, previous):
        diff = current - previous

        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi

        return diff

    def timer_callback(self):
        """
            Timer callback for autonomous object image capture.

            When the robot enters the 'taking picture' state, this callback:
                1. Continuously acquires RGB frames from the camera.
                2. Runs object detection inference on each frame.
                3. Filters detections using:
                    - minimum confidence threshold
                    - minimum bounding box size
                4. Selects the most relevant object candidate.
                5. Rotates the robot in place using Twist commands until the
                   object is centered in the image frame.
                6. Applies temporal stability checks across multiple frames to
                   avoid transient detections.
                7. Stops robot motion once a stable centered target is achieved.
                8. Captures and saves an image of the detected object.
                9. Publishes a completion flag to notify the global
                   controller that image acquisition is complete.

            This callback also publishes annotated detection images

            Assumptions:
                - The mission/controller node handles waypoint generation and navigation.
                - This node is responsible only for local visual alignment and image capture.
                - The robot is capable of differential-drive turning in place.

        """

        #protect against this locking out/fighting global controller if needed
        if self.master_status != 'taking picture':
            return


        #stop searching after 360 spin
        if self.total_yaw >= self.full_rotation_threshold:
            self.get_logger().warn("360 scan complete, no valid object found")

            stop = Twist()
            self.cmd_vel_pub.publish(stop)

            self.searching = False
            self.yaw_initialised = False
            self.total_yaw = 0.0

            fail_msg = Bool()
            fail_msg.data = False
            self.scan_complete_pub.publish(fail_msg)

            self.master_status = "idle"

            return

        if self.frame is None:
            return
        else:
            currFrame = self.frame
            currResults = self.results

        if not self.searching:
            self.searching = True
            self.total_yaw = 0.0
            self.yaw_initialised = False

        frame_h, frame_w = currFrame.shape[:2]
        frame_center_x = frame_w / 2.0
        best_detection = None
        best_area = 0

        for result in currResults:
            if result.boxes is None:
                continue

            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                box_width = x2 - x1
                box_height = y2 - y1
                area = box_width * box_height

                if area < self.min_box_area:
                    continue

                center_x = (x1 + x2) / 2.0

                if area > best_area:
                    best_area = area

                    #get class index and convert to label
                    cls = int(box.cls[0].cpu().numpy())
                    classification = self.label_map.get(cls, str(cls))

                    #store class lookup
                    best_detection = {
                        'classification': classification,
                        'x1': x1,
                        'y1': y1,
                        'x2': x2,
                        'y2': y2,
                        'center_x': center_x,
                        'area': area
                    }

        cmd = Twist()

        if best_detection is None:
            self.target_locked_frames = 0
            cmd.angular.z = 0.25
            self.cmd_vel_pub.publish(cmd)
            return

        #target found, try to centre - this may be optimistic, may need to add scoring metric as a function of area and distance too camera
        target_error = best_detection['center_x'] - frame_center_x
        kP = 0.0025
        cmd.angular.z = -kP * target_error
        cmd.angular.z = max(min(cmd.angular.z, 0.3), -0.3)

        self.cmd_vel_pub.publish(cmd)

        centered = abs(target_error) < self.center_threshold_px

        #stay centered for a few frames to combat false postives
        if centered:
            self.target_locked_frames += 1
        else:
            self.target_locked_frames = 0

        x1 = best_detection['x1']
        y1 = best_detection['y1']
        x2 = best_detection['x2']
        y2 = best_detection['y2']

        #draw bounding box
        cv2.rectangle(currFrame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        #draw centre line
        cv2.line(currFrame, (int(frame_center_x), 0), (int(frame_center_x), frame_h), (255, 0, 0), 2)

        #label object
        detected_class = best_detection['classification']
        cv2.putText(
            currFrame,
            f"Locked: {detected_class}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )

        if self.target_locked_frames >= self.required_locked_frames:

            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

            timestamp = int(time.time())

            filename = f"{self.imgdir}/{detected_class}_{timestamp}.jpg"
            cv2.imwrite(filename, currFrame)
            self.get_logger().info(f"Saved image: {filename}")

            #inform controller
            done_msg = Bool()
            done_msg.data = True

            self.scan_complete_pub.publish(done_msg)

            self.target_locked_frames = 0
            self.master_status = 'idle'

        detection_img_msg = self.bridge.cv2_to_imgmsg(currFrame, encoding="bgr8")
        time_now = self.get_clock().now().to_msg()

        detection_img_msg.header.stamp = time_now

        detection_img_msg.header.frame_id = "camera_link"
        self.image_pub.publish(detection_img_msg)

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